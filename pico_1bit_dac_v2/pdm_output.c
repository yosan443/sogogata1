/**
 * @file pdm_output.c
 * @brief ハイレゾ改良版(HR2/V2)
 * @author geachlab, Yasushi MARUISHI
 * @version 0.4
 * @date 2023.02.21
 * @note
 * 0.3 改良点： 
 * 旧処理 : Core0:オーバーサンプリングf特補正 ～ Core1:2次曲線補間 x64オーバーサンプリング　～　3次ΔΣ+4bitPWM
 * 新処理 : Core0:3連結ハーフバンド x8オーバーサンプリング ～ Core1:x8ゼロホールド　～ 4次ΔΣ+5bitPWM or 5次ΔΣ+6bitPWM
 * ・Core0側に x2~x8オーバーサンプリングを実装、Core1側への入力は fs = 352.8/384kHzに統一。
 * 　Core1側のオーバーサンプリングはx8固定で場合分け不要となり、処理が単純化。
 * 　CPU処理時間に余裕ができ、4次・5次ΔΣ演算が可能となった。
 * 0.4 GPIO Drive strength 初期化ミス修正 GP14,GP16のみ設定→GP14~17を設定
 *     pdm fs設定をbsp.cに移設　
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include "pico/stdlib.h"
//#include "pico/platform.h"
#include "hardware/interp.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "bsp.h"
#include "simple_queue.h"

#if 0 /*PWM/PDM処理時間計測時に使用*/
#define DEBUG_PIN_PUT(x, y) gpio_put((x), (y))
#define DEBUG_PIN_SET(pin) gpio_set_mask((1u << (pin)))
#define DEBUG_PIN_CLR(pin) gpio_clr_mask((1u << (pin)))
#else
#define DEBUG_PIN_PUT(x, y)	/*処理なし*/
#define DEBUG_PIN_SET(pin)	/*処理なし*/
#define DEBUG_PIN_CLR(pin)	/*処理なし*/
#endif

#define PWM_BIT	6	// PWM分解能(4~6) 4~5:4~5bit(cycle = 3.072M) 6:6bit(cycle = 1.536M)
#define DS_ORDER 5	// ΔΣ次数(0~5) 設定値は PWM_BIT > DS_ORDER とすること
#define OS_TYPE 1	// x8オーバサンプラ動作選択 0:SH(SampleHold) 1:LinerInterpolator(直線補間)

// 定数群
const uint	os_inner_loop_n = 4;			// Interpで処理するループ回数(bit長にかかわらず4回に固定) 
const uint	ds_bitshift = 7;				// ΔΣ処理ビット長 ― 音源ビット長 = 31 - 24 = 7
const uint32_t pwm_mask = ((int32_t)0x80000000) >> (PWM_BIT - 1);	// Ex. PWM_BIT = 5 ; pwm_mask = 0xf8000000
const uint32_t pwm_bitshift = 32 - PWM_BIT * os_inner_loop_n;	// full bit - working pwm bit 

#if   (PWM_BIT == 4)
  #include "pio_pwm_4bit.pio.h"
  const uint os_bitshift = 3;	// 2^os_bitshift = x8
  const uint os_outer_loop_n = 8 / os_inner_loop_n;	// x8 OverSampling / 8
#elif (PWM_BIT == 5)
  #include "pio_pwm_5bit.pio.h"
  const uint os_bitshift = 3;	// 2^os_bitshift = x8
  const uint os_outer_loop_n = 8 / os_inner_loop_n;	// x8 OverSampling / 4
#elif (PWM_BIT == 6)
  #include "pio_pwm_6bit.pio.h"
  const uint os_bitshift = 2;	// 2^os_bitshift = x4
  const uint os_outer_loop_n = 4 / os_inner_loop_n;	// x4 OverSampling / 4
#endif

#define DS_MAX	(DS_ORDER + 1)	// ΔΣレジスタワークの最大数(最大のΔΣ次数)
#define BS_MAX	2				// ビットストリームデータ最大段数

// PWM変換処理構造体定義・宣言
typedef struct {
	uint32_t	ds[DS_MAX];		// ΔΣレジスタワーク 処理終了時に退避、処理再開時に復帰利用 OB(Offset Binary)処理に伴い int->uintに変更
	uint32_t	bs[BS_MAX];		// ビットストリーム 時刻順：LSB First,pdm[0]->pdm[1]
	int32_t		d1;				// 前回入力データ 線形補間用
} pcm2pwm_arg_t;

static pcm2pwm_arg_t ch0;	// L/R Channel PWM変換処理構造体
static uint32_t ds_pwm_offset;	// ΔΣ/PWM OB(Offset Binary)演算用加算値

// PWM変換初期化
// PWM変換処理構造体のゼロクリアを行う
void pcm2pwm_reset(void){
	// 遅延データクリア ゼロフィル
	ch0.d1 = 0;
	// ΔΣワーククリア
	for(uint k = 0; k < DS_MAX; k++){
		ch0.ds[k] = 0;
	}
}

// PWM変換処理初期化
void pcm2pwm_init(uint pwm_bit){

	// DS/PWM OB演算用オフセット
	// MSB=1で加算で符号反転する PWMタイプによっては
	// 中心偏差が必要なためグローバル変数化している
	ds_pwm_offset = 0x80000000;

	interp_config cfg;
	// Interp0 Lane0 : オーバーサンプラ
	// 8fs(384k入力)に対し SH(Sample Hold)/直線補間を
	// 行い 32fs(1.576M)/64fs(3.072M)データを得る
	cfg = interp_default_config();                  // set default config
	interp_config_set_add_raw(&cfg, true);          // Use ADD_RAW path (as accum[0] += base[0]) 
	interp_config_set_shift(  &cfg, 0);             // Setting the bit width increased by the Interp processing
	interp_config_set_mask(   &cfg, 0, 31);         // Setting the effective bit mask width for Interp processing
	interp_config_set_signed( &cfg, false);         // Not Use sign-extended
	interp_set_config(interp0, 0, &cfg);            // Set interp0 lane0
	interp0->accum[0] = 0;                          // Reset
	interp0->base[0] = 0;                           // Reset
	interp0->base[2] = ds_pwm_offset;               // DS/PWM OB演算用オフセット

	// Interp0 Lane1 : アイドルトーン拡散
	// 微弱な矩形波(24bitデータのLSB以下)を生成、
	// オーバーサンプリング中のデータに重畳しアイドルトーンを拡散する。
	cfg = interp_default_config();                  // set default config
	interp_config_set_add_raw(&cfg, true);          // Use accum[1]-Shift-Mask-sign-add base[1]-accum[1] path
	interp_config_set_shift(  &cfg, 0);             // 0-bit Right shift 
	interp_config_set_mask(   &cfg, 6, 6);          // Mask Only bit 6
	interp_config_set_signed( &cfg, false);         // Use sign-extended
	interp_set_config(interp0, 1, &cfg);            // Set interp0 lane1
	interp0->base[1] = 1;                           // Delta Data
	interp0->accum[1]= 0;                           // Start Data

	////////////////////////////////////////////////// interp1 lane0 : the quantizer
	cfg = interp_default_config();                  // set default config
	interp_config_set_cross_input(&cfg, true);      // Use accum[1] for lane0. Note:The quantizer (result[0]) is delayed by 1clk. It should be read twice by pop and pull.
	interp_config_set_add_raw(&cfg, false);         // Use accum[1]-Shift-Mask-Signed-add base[0] path
	interp_config_set_shift(&cfg, 0);               // 0-bit Right Shift
	interp_config_set_mask(&cfg, 32 - pwm_bit, 31);	// mask Upper PWM_BIT
	interp_config_set_signed(&cfg, false);          // Not used sign-extended
	interp_set_config(interp1, 0, &cfg);            // Set interp1 lane0
	interp1->base[0]=(uint32_t)0x80000000>>pwm_bit; // Quantizer Offset

	////////////////////////////////////////////////// interp1 lane1 : the bitstreamer
	cfg = interp_default_config();                  // set default config
	interp_config_set_add_raw(&cfg, false);         // Use accum[1]-Shift-Mask-unsigned-add base[1]-accum[1] path
	interp_config_set_shift(&cfg, pwm_bit);         // N-bit shift for bitstream recording 
	interp_config_set_mask(&cfg, 0, 31 - pwm_bit);  // Unmask Upper N-bit for recorded bitstream
	interp_config_set_signed(&cfg, false);          // Not used sign-extended
	interp_set_config(interp1, 1, &cfg);            // Set interp1 lane1
	interp1->base[1]  = 0;                          // reset base[1]
	interp1->base[2]  = 0;                          // not used base[2]

	pcm2pwm_reset();
}


static inline void pcm2pwm(int32_t d0, pcm2pwm_arg_t *ch)
{
// x8オーバーサンプラー初期設定
#if (OS_TYPE == 0)		// SH(サンプルホールド型)
  interp0->accum[0] = d0 << ds_bitshift;
  interp0->base[0] = 0;
#elif (OS_TYPE == 1)	// Liner-Interpolator(直線補間型)
  interp0->accum[0] = ch->d1 << ds_bitshift;
  interp0->base[0] = (d0 - ch->d1) << (ds_bitshift - os_bitshift);
  ch->d1 = d0;
#endif

  // Pre_Process : Set previous delta-sigma values to interp
  interp1->base[1] = ch->ds[0];

  uint32_t qt_out;
    // Delta-Sigma & Bitstream Process 
    for(uint j = 0; j < os_outer_loop_n; j++){
      for(uint k = 0; k < os_inner_loop_n; k++){
        // Dummy read & Get Quantized Data 
        qt_out = interp1->pop[0];
        qt_out = interp1->peek[0];
#if   (DS_ORDER == 0)	// ΔΣなし
        // d/s (delta-sigma) 
        // set d/s data to Bitstreamer/Quantizer
        interp1->base[1] = interp0->pop[2] & pwm_mask;
#elif (DS_ORDER == 1)	// 1次ΔΣ
        // d/s (delta-sigma) 
        ch->ds[1] += -qt_out +interp0->pop[2];
        // set d/s data to Bitstreamer/Quantizer
        interp1->base[1] = ch->ds[1] & pwm_mask;
#elif (DS_ORDER == 2)	// 2次ΔΣ
        // d/s (delta-sigma) 
        ch->ds[1] += -qt_out +interp0->pop[2];
        ch->ds[2] += -qt_out +ch->ds[1];
        // set d/s data to Bitstreamer/Quantizer
        interp1->base[1] = ch->ds[2] & pwm_mask;
#elif (DS_ORDER == 3)	// 3次ΔΣ
        // d/s (delta-sigma) 
        ch->ds[1] += -qt_out +interp0->pop[2];
        ch->ds[2] += -qt_out +ch->ds[1];
        ch->ds[3] += -qt_out +ch->ds[2];
        // set d/s data to Bitstreamer/Quantizer
        interp1->base[1] = ch->ds[3] & pwm_mask;
#elif (DS_ORDER == 4)	// 4次ΔΣ
        // d/s (delta-sigma) 
        ch->ds[1] += -qt_out +interp0->pop[2];
        ch->ds[2] += -qt_out +ch->ds[1];
        ch->ds[3] += -qt_out +ch->ds[2];
        ch->ds[4] += -qt_out +ch->ds[3];
        // set d/s data to Bitstreamer/Quantizer
        interp1->base[1] = ch->ds[4] & pwm_mask;
#elif (DS_ORDER == 5)	// 5次ΔΣ
        // d/s (delta-sigma) 
        ch->ds[1] += -qt_out +interp0->pop[2];
        ch->ds[2] += -qt_out +ch->ds[1];
        ch->ds[3] += -qt_out +ch->ds[2];
        ch->ds[4] += -qt_out +ch->ds[3];
        ch->ds[5] += -qt_out +ch->ds[4];
        // set d/s data to Bitstreamer/Quantizer
        interp1->base[1] = ch->ds[5] & pwm_mask;
#endif
      }
      // get final PWM Data(4-data/32bit)
      ch->bs[j] = (interp1->peek[1] >> pwm_bitshift);
    }
    // Post process : Save final delta-sigma values
    ch->ds[0] = interp1->base[1];
}

// PWM出力ピンのPAD初期化
void pwm_gpio_init()
{
	// PWM GPIO Pad Setting
	for(uint ch = 0; ch < 2; ch++){
		uint pin = (ch == 0) ? PIN_OUTPUT_LP : PIN_OUTPUT_RP;
		for(uint i = 0; i < 2; i++){
			gpio_set_input_enabled(pin, 0);							// 0:Disable,1:Enable
			gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);			// _FAST, _SLOW
			// drive_strength を可変すると、N次歪みの出方が変わる
			// 現状では最も歪みの少なかった2mA設定とした
			gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_2MA);	// _2MA,_4MA,_8MA,_12MA
			gpio_set_pulls(pin, 0, 0);								// 0:Disable,1:Enable
			pin ++;
		}
	}
}

// pio0 sm0/sm1 fifo 最適化アクセス関数
// sm0/sm1引数チェック削除とFIFOチェック統合により
// sm0/sm1間の設定ラグタイムを極力排除する
#define SM0_SM1_TXFULL (3u << PIO_FSTAT_TXFULL_LSB)
static inline void pio0_sm01_put_blocking(
	uint32_t ch0_data,
	uint32_t ch1_data
){
#if 1	// 高速アクセス版
	// sm0/sm1 FIFO FULL 解除待ちループ
	while((pio0->fstat & SM0_SM1_TXFULL) != 0){
		tight_loop_contents();
	}
	// sm0/sm1 FIFO レジスタへの連続書き込み
	pio0->txf[0] = ch0_data;	
	pio0->txf[1] = ch1_data;
#else	// 従来版
	pio_sm_put_blocking(pio0, 0, ch0_data);
	pio_sm_put_blocking(pio0, 1, ch1_data);
#endif
}

static inline void pio0_sm0_put_blocking(uint32_t ch0_data)
{
    while((pio0->fstat & PIO_FSTAT_TXFULL_BITS) != 0){
        tight_loop_contents();
    }
    pio0->txf[0] = ch0_data;
}

// 再生処理
void pdm_output()
{
    static bool mute_flag = false;
	int32_t mute_buff[24*2] = {0};  // 無音buff 通常buff長(384*2)の1/16(62.5us)

	pio_pwm_program_init(pio0, PIN_OUTPUT_LP, PIN_OUTPUT_RP, PIN_FS48);
	pcm2pwm_init(PWM_BIT);
	pwm_gpio_init();

    while(1){

/* <Mute Logic>
         queue_length     A    *                 *         
                [ms]      |     *               *          
                          | _ _ _*_ _ _ _ _ _ _*_ _ _ _ _ _ QUEUE_PLAY_THR
                        A |       *           *:           
                Mute    | |        *         * :           
            Hysteresis  | |         *       *  :           
                [ms]    V |          *     *   :           
                        0 |___________*****____:___________
                                      :        :           
            mute_flag     ____________~~~~~~~~~____________
                                      :        :
            action         <---play-->:<-mute->:<---play-->

		ミュート有効・解除切り替えは、キュー長によりヒステリシスを持たせている。
		ミュート有効時はキュー長が充足(queue_length ≧ QUEUE_PLAY_THR)するまでミュート有効を保持する。
		キュー長が充足したらミュート解除(mute_flag=0)し、キューの再生処理(dequeue～pcm2pwm~PIO)を行う。
		以降、キュー長がゼロ(queue_length = 0)となるまでミュート解除状態を保持し、キューを再生しきる。
		本プログラムでは、再生ディレイと安定のバランスを考慮し、QUEUE_PLAY_THR = 4 (≒4ms)とした。
		キュー長が一定値となるための制御は、Core0側の周波数Feedback側に実装されている。
*/
		// ミュート判定
		uint32_t queue_length = get_queue_length();
		if((queue_length == 0)&&(mute_flag == false))	// mute開始条件段数
		{
			mute_flag = true;
			queue_reset();
			pcm2pwm_reset();
		}
		else if(queue_length >= QUEUE_PLAY_THR)			// mute解除条件段数
		{
			mute_flag = false;
		}

		// バッファ宣言・指定 初期状態をミュートバッファとしておく
		int32_t* buff = mute_buff;
		uint32_t len = sizeof(mute_buff) / (sizeof(int32_t) * 2);
#if 0
		// ミュート解除時のキューバッファ位置取得
		if(mute_flag == false){
			dequeue(&buff, &len);	// キューbuff/len取得　失敗時は buff/lenは更新されずミュートバッファのままとなる
		}
#endif 
		// 再生処理本体：
		// 万が一PIOにデータが供給されない場合でも、自動的にPIO内の無音データが再生される。
		if(mute_flag == false)
		{
			dequeue(&buff, &len);	// キューbuff/len取得　失敗時は buff/lenは更新されずミュートバッファのままとなる
			DEBUG_PIN_SET(PIN_TIME_MEASURE);		// テスト用 オシロ観測用トリガ PCMデータ先頭で1
			while(len--){		// Buffer loop
				pcm2pwm(*buff++, &ch0);			// LCh PWM変換 結果は構造体 ch[].bs (bitstream)に代入される
				DEBUG_PIN_SET(PIN_PIOT_MEASURE);	// テスト用 pio設定前にH。pioに待たされている時刻測定用
				for(uint k = 0; k < os_outer_loop_n; k ++){
					pio0_sm0_put_blocking(ch0.bs[k]);	// LCh/RCh Bitstream を PIO PWMへ出力
				}
				DEBUG_PIN_CLR(PIN_PIOT_MEASURE);	// テスト用 pio設定後にL。pioに待たされている時刻測定用
			}
			DEBUG_PIN_CLR(PIN_TIME_MEASURE);		// テスト用 オシロ観測用トリガ PCMデータ先頭以外で0
		}
	}
}