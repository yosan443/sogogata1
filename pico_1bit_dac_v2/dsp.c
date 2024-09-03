/**
 * @file dsp.c
 * @author geachlab, Yasushi MARUISHI
 * @adder Yoshimi Sugawara
 * @brief dsp処理関数群、周辺関数
 * @version 0.01Y
 * @date 2024-08-27
 * @note 旧名称 oversampler.c
 *       dsp処理を集結 : oversampler, volume, asrc 周波数管理関数等  
 * @add_note 今回の開発する機器の目的に合わせ、一部記述を削除。加筆部分は行末に////と説明を追加。無効化部分は////でコメントアウト。
 */

// 連結ハーフバンドフィルタの処理時間計測時に1とする
#if 0
#define DEBUG_PIN(x, y) gpio_put((x), (y))
#else
#define DEBUG_PIN(x, y)	/*処理なし*/
#endif

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/interp.h"

#include "pins.h"
#include "simple_queue.h"

// クランプ幅:24.5bit (3.52dBFS 最大入力振幅の±1.5倍)
#define CLAMP_MAX ((+1 << 23) + (+1 << 22) - 1)
#define CLAMP_MIN ((-1 << 23) + (-1 << 22) )

// Interp1 ハードクランプ初期化
// 各オーバーサンプラのフィルタ処理後のレベルクリップに利用
void interp1_hw_clamp_init(void){

	// デフォルト値取得
	interp_config cfg = interp_default_config();
	
	// クランプモード使用
	interp_config_set_clamp( &cfg, true);
	
	// shift未使用、マスク未使用、符号拡張使用
	interp_config_set_shift( &cfg, 0);
	interp_config_set_mask( &cfg, 0,31);
	interp_config_set_signed( &cfg, true);
	
	// Interp lane0設定
	interp_set_config(interp1, 0, &cfg);

	// Interpレジスタ初期値設定
	interp1->base[0] = CLAMP_MIN; // クランプ最小値
	interp1->base[1] = CLAMP_MAX; // クランプ最大値
	interp1->base[2] = 0;         // 未使用
	interp1->accum[1] = 0;        // 未使用
}

inline int32_t clamp(int32_t x){
#if 1	// ハードウェアclamp (interp1利用)
	interp1->accum[0] = x;
	return interp1->peek[0];
#else	// ソフトウェアclamp
  #if 1 
	return (x > CLAMP_MAX) ? CLAMP_MAX : (x < CLAMP_MIN) ? CLAMP_MIN : x;
  #else	// clamp実行量観測用
	if        (x > CLAMP_MAX) {
		gpio_put(PIN_DEBUG_GP10, 1);
		x = CLAMP_MAX;
		gpio_put(PIN_DEBUG_GP10, 0);
	} else if (x < CLAMP_MIN) {
		gpio_put(PIN_DEBUG_GP11, 1);
		x = CLAMP_MIN;
		gpio_put(PIN_DEBUG_GP11, 0);
	}
	return x;
  #endif
#endif
}


// Interp0 ブレントモード初期化
// ASRCのサンプル間αブレンドに使用
void interp0_blender_init(void){

	// default値取得, blendモード使用, Interp0 lane0設定
	interp_config cfg = interp_default_config();
	interp_config_set_blend( &cfg, true);
	// shift未使用、マスク未使用、符号拡張使用
//	interp_config_set_shift( &cfg, 0);
//	interp_config_set_mask( &cfg, 0,31);
	interp_set_config(interp0, 0, &cfg);
	
	// default値取得, Interp0 lane1設定
	cfg = interp_default_config();
	interp_config_set_signed( &cfg, true);
	interp_set_config(interp0, 1, &cfg);

	// Interpレジスタ初期値設定
	interp0->base[2] = 0;         // 未使用
	interp0->accum[0] = 0;        // 未使用
}


// 48kHz系列フラグ取得 fs=44.1k~768kの範囲で下位7bitが必ず0である性質を利用
bool get_group_48k(uint fs){
	return ((fs & 0x7f) == 0);
}

// OSR(Over Sampling Rate)値の算出
uint get_osr(uint fs){
	if ((fs & 0x7f) == 0) return fs / 48000;
	else                  return fs / 44100;
}

// 真の再生周波数取得
// DAC再生周波数はシステムクロック(clk_sys)の整数分周で生成するため、理想周波数に対し誤差を持つ。この周波数を取得する。
float get_true_playback_fs(uint fs){
	switch(fs) {
//								clk_sys   div   ratio		true fs				error	
		case 384000:	return (CLK_SYS / 68.0 /  8);	// = 383823.529...Hz, -460ppm
		case 352800:	return (CLK_SYS / 74.0 /  8);	// = 352702.702...Hz, -276ppm
		case 192000:	return (CLK_SYS / 68.0 / 16);	// = 191911.764...Hz, -460ppm
		case 176400:	return (CLK_SYS / 74.0 / 16);	// = 176351.351...Hz, -276ppm
		case  96000:	return (CLK_SYS / 68.0 / 32);	// =  95955.882...Hz, -460ppm
		case  88200:	return (CLK_SYS / 74.0 / 32);	// =  88175.675...Hz, -276ppm
		case  48000:	return (CLK_SYS / 68.0 / 64);	// =  47977.941...Hz, -460ppm
		case  44100:
		default:		return (CLK_SYS / 74.0 / 64);	// =  44087.837...Hz, -276ppm
	}
}

#define HBF3_TAP_N	11						// HBFフィルタの元のタップ数
#define HBF3_ITAP_N	((HBF3_TAP_N + 1) / 2)	// 補間用フィルタ(φ1)のタップ数
void hbf3_x2_oversampler(
	int32_t *p_i,		// 24bit input data pointer  : L1,R1,L2,R2,L3,R3,,Ln,Rn (n=*p_len)
	int32_t *p_o,		// 24bit output data pointer : L1,R1,L2,R2,L3,R3,,Lm,Rm (m=*p_len *2)
	uint *p_len			// *p_len > 0 : num. of sample, *p_len = 0 : Reset Oversampler
){
	const uint k_bit_w = 7;								// 固定係数ビット長定義
	const int32_t k[HBF3_ITAP_N / 2] = {
//		-10, +74/*, +74, -10*/};						// 固定係数定義 偶数番はゼロのため省略、左右対称のため後半省略
		2, -14, +76/*, +76, -14, 2*/};					// 固定係数定義 偶数番はゼロのため省略、左右対称のため後半省略
	static uint t = 0;									// 遅延タップ位置宣言 連続利用のためstaticとする
	static int32_t z[HBF3_ITAP_N * 4];					// 遅延データ列宣言 連続利用のためstaticとする
	uint len = *p_len;									// ローカル変数に処置ループ長を取得

	if (len == 0) {										// 遅延データ列とタップ位置をリセット
		t = 0;
		for(uint c = 0; c < HBF3_ITAP_N * 4; c++) z[c] = 0;
	} else {											// オーバーサンプル処理
		while(len--){
			////////////////////////////////////////////// Ch0 Oversampler
			int32_t d = *p_i++;							// 入力データ取得
			z[t               ] = d;					// 第1遅延データ更新
			z[t + HBF3_ITAP_N ] = d;					// 第2遅延データ更新
			*p_o = z[t + HBF3_ITAP_N / 2];				// 中央遅延データを実データとして出力
			p_o	+= 2;									// 出力ポインタを補間データ位置へ移動

			// 補間データ演算
			d  = k[ 0] * (int32_t)(d        +z[t + 5]);	// d  = k0*(tap0 + tap10)
			d += k[ 1] * (int32_t)(z[t + 1] +z[t + 4]);	// d += k1*(tap2 + tap8)
			d += k[ 2] * (int32_t)(z[t + 2] +z[t + 3]);	// d += k1*(tap4 + tap6)
														// 演算結果は32bit幅に収まるため64bit演算に移行する必要なし
			d = clamp(d >> k_bit_w);					// 演算結果を係数ビット長で右シフトし指定値でクランプし補間データ完成
			*p_o--	= d;								// 補間データを出力、出力ポインタをCh1実データ位置へ移動
			t += 2 * HBF3_ITAP_N;						// タップ位置をCh1部に移動

			////////////////////////////////////////////// Ch1 Oversampler
			d = *p_i++;									// 入力データ取得
			z[t               ] = d;					// 第1遅延データ更新
			z[t + HBF3_ITAP_N ] = d;					// 第2遅延データ更新
			*p_o = z[t + HBF3_ITAP_N / 2];				// 中央遅延データを実データとして出力
			p_o	+= 2;									// 出力ポインタを補間データ位置へ移動

			// 補間データ演算
			d  = k[ 0] * (int32_t)(d        +z[t + 5]);	// d  = k0*(tap0 + tap10)
			d += k[ 1] * (int32_t)(z[t + 1] +z[t + 4]);	// d += k1*(tap2 + tap8)
			d += k[ 2] * (int32_t)(z[t + 2] +z[t + 3]);	// d += k1*(tap4 + tap6)
														// 演算結果は32bit幅に収まるため64bit演算に移行する必要なし
			d = clamp(d >> k_bit_w);					// 演算結果を係数ビット長で右シフトし指定値でクランプし補間データ完成
			*p_o++	= d;								// 補間データを出力、出力ポインタをCh0実データ位置へ移動
			t -= 2 * HBF3_ITAP_N;						// タップ位置をCh0部に移動

			if (t == 0)	t = HBF3_ITAP_N - 1;			// タップが先頭に戻ったら最終タップに戻す
			else		t--;							// 1タップずらす
		}
	}
	*p_len *= 2;										// オーバーサンプリングで倍増したデータ数に更新
}


#define HBF2_TAP_N	15						// HBFフィルタの元のタップ数
#define HBF2_ITAP_N	((HBF2_TAP_N + 1) / 2)	// 補間用フィルタ(φ1)のタップ数
void hbf2_x2_oversampler(
	int32_t *p_i,		// 24bit input data pointer  : L1,R1,L2,R2,L3,R3,,Ln,Rn (n=*p_len)
	int32_t *p_o,		// 24bit output data pointer : L1,R1,L2,R2,L3,R3,,Lm,Rm (m=*p_len *2)
	uint *p_len			// *p_len > 0 : num. of sample, *p_len = 0 : Reset Oversampler
){
	const uint k_bit_w = 10;							// 固定係数ビット長定義
	const int32_t k[HBF2_ITAP_N / 2] = {
		  -8, +43,-149,+626/*,+626,-149, +43,  -8*/};	// 固定係数定義 偶数番は0,後半は左右対称のため省略
	static uint t = 0;									// 遅延タップ位置宣言 連続利用のためstaticとする
	static int32_t z[HBF2_ITAP_N * 4];  				// 遅延データ列宣言 連続利用のためstaticとする
	uint len = *p_len;									// ローカル変数に処置ループ長を取得

	if (len == 0) {										// 遅延データ列とタップ位置をリセット
		t = 0;
		for(uint c = 0; c < HBF2_ITAP_N * 4; c++) z[c] = 0;
	} else {											// オーバーサンプル処理
		while(len--){
			////////////////////////////////////////////// Ch0 Oversampler
			int32_t d = *p_i++;							// 入力データ取得
			z[t              ] = d;						// 第1遅延データ更新
			z[t + HBF2_ITAP_N] = d;						// 第2遅延データ更新
			*p_o = z[t + HBF2_ITAP_N / 2];				// 中央遅延データを実データとして出力
			p_o	+= 2;									// 出力ポインタを補間データ位置へ移動

			// 補間データ演算
			d  = k[ 0] *          (d        +z[t + 7]);	// k0 * (tap0 + tap14)
			d += k[ 1] *          (z[t + 1] +z[t + 6]);	// k2 * (tap2 + tap12)
			int64_t x  = (int64_t)d;					// 以降演算結果が32bit幅を超えるため64bitに乗り換え
			x += k[ 2] * (int64_t)(z[t + 2] +z[t + 5]);	// k4 * (tap4 + tap10)
			x += k[ 3] * (int64_t)(z[t + 3] +z[t + 4]);	// k6 * (tap6 + tap8)
			d = clamp(x >> k_bit_w);					// 64bit長演算結果を係数ビット長で右シフトし32bit幅に戻した後、指定値でクランプし補間データ完成
			*p_o--	= d;								// 補間データを出力、出力ポインタをCh1実データ位置へ移動
			t += 2 * HBF2_ITAP_N;						// タップ位置をCh1部に移動

			////////////////////////////////////////////// Ch1 Oversampler
			d = *p_i++;									// 入力データ取得
			z[t              ] = d;						// 第1遅延データ更新
			z[t + HBF2_ITAP_N] = d;						// 第2遅延データ更新
			*p_o = z[t + HBF2_ITAP_N / 2];				// 中央遅延データを実データとして出力
			p_o	+= 2;									// 出力ポインタを補間データ位置へ移動

			// 補間データ演算
			d  = k[ 0] *          (d        +z[t + 7]);	// k0 * (tap0 + tap14)
			d += k[ 1] *          (z[t + 1] +z[t + 6]);	// k2 * (tap2 + tap12)
			x  = (int64_t)d;							// 以降演算結果が32bit幅を超えるため64bitに乗り換え
			x += k[ 2] * (int64_t)(z[t + 2] +z[t + 5]);	// k4 * (tap4 + tap10)
			x += k[ 3] * (int64_t)(z[t + 3] +z[t + 4]);	// k6 * (tap6 + tap8)
			d = clamp(x >> k_bit_w);					// 64bit長演算結果を係数ビット長で右シフトし32bit幅に戻した後、指定値でクランプし補間データ完成
			*p_o++	= d;								// 補間データを出力、出力ポインタをCh0実データ位置へ移動
			t -= 2 * HBF2_ITAP_N;						// タップ位置をCh0部に移動

			if (t == 0)	t = HBF2_ITAP_N - 1;			// タップが先頭に戻ったら最終タップに戻す
			else		t--;							// 1タップずらす
		}
	}
	*p_len *= 2;										// データ長がオーバーサンプリングで倍増するため更新
}


#define HBF1_TAP_N	31
#define HBF1_ITAP_N	((HBF1_TAP_N + 1) / 2)
void hbf1_x2_oversampler(
	int32_t *p_i,		// 24bit input data pointer  : L1,R1,L2,R2,L3,R3,,Ln,Rn (n=*p_len)
	int32_t *p_o,		// 24bit output data pointer : L1,R1,L2,R2,L3,R3,,Lm,Rm (m=*p_len *2)
	uint *p_len			// *p_len > 0 : num. of sample, *p_len = 0 : Reset Oversampler
){
	const uint k_bit_w = 12;
	const int32_t k[HBF1_ITAP_N /2] = {
		-2,+10,-32,+81,-177,+360,-762,+2570};			// 12-bit 16tap
	static uint t = 0;
	static int32_t z[HBF1_ITAP_N * 4];
	uint len = *p_len;
	if (len == 0) {
		// Clear Delayed Data
		t = 0;
		for(uint c=0; c < HBF1_ITAP_N*4; c++) z[c] = 0;
	} else {
		while(len--){
			// Ch0 Oversampler
			int32_t d = *p_i++;
			z[t              ] = d;
			z[t + HBF1_ITAP_N] = d;
			*p_o = z[t + HBF1_ITAP_N / 2];
			p_o	+= 2;

			d  = k[ 0] * (int32_t)(d        +z[t +15]);
			d += k[ 1] * (int32_t)(z[t + 1] +z[t +14]);
			d += k[ 2] * (int32_t)(z[t + 2] +z[t +13]);
			d += k[ 3] * (int32_t)(z[t + 3] +z[t +12]);
			int64_t x  = (int64_t)d;
			x += k[ 4] * (int64_t)(z[t + 4] +z[t +11]);
			x += k[ 5] * (int64_t)(z[t + 5] +z[t +10]);
			x += k[ 6] * (int64_t)(z[t + 6] +z[t + 9]);
			x += k[ 7] * (int64_t)(z[t + 7] +z[t + 8]);
			*p_o--	= clamp(x >> k_bit_w);
			t += 2 * HBF1_ITAP_N;

			// Ch1 Oversampler
			d = *p_i++;
			z[t              ] = d;
			z[t + HBF1_ITAP_N] = d;
			*p_o = z[t + HBF1_ITAP_N / 2];
			p_o	+= 2;

			d  = k[ 0] * (int32_t)(d        +z[t +15]);
			d += k[ 1] * (int32_t)(z[t + 1] +z[t +14]);
			d += k[ 2] * (int32_t)(z[t + 2] +z[t +13]);
			d += k[ 3] * (int32_t)(z[t + 3] +z[t +12]);
			x  = (int64_t)d;
			x += k[ 4] * (int64_t)(z[t + 4] +z[t +11]);
			x += k[ 5] * (int64_t)(z[t + 5] +z[t +10]);
			x += k[ 6] * (int64_t)(z[t + 6] +z[t + 9]);
			x += k[ 7] * (int64_t)(z[t + 7] +z[t + 8]);
			*p_o++	= clamp(x >> k_bit_w);
			t -= 2 * HBF1_ITAP_N;
			
			if (t == 0)	t = HBF1_ITAP_N - 1;
			else		t--;
		}
	}
	*p_len *= 2;
}

// 各オーバーサンプラのリセット
// フィルタ内の遅延データを消去し、ノイズ発生を防ぐ
void hbf_oversampler_reset(void){
	int32_t null_buf[2];
	uint null_len = 0;
	hbf1_x2_oversampler(null_buf, null_buf, &null_len);
	hbf2_x2_oversampler(null_buf, null_buf, &null_len);
	hbf3_x2_oversampler(null_buf, null_buf, &null_len);
}

// 音量処理関数
// 従来の64bit演算を32bit化し高速化を行っている
/*
void volume(int32_t* buf, uint32_t sample_num, int32_t mul, uint shift){
	int32_t d;			// work data
	while(sample_num --){
		d = *buf;	*buf++ = (d * mul) >> shift;	// Volume処理
		d = *buf;	*buf++ = (d * mul) >> shift;	// Volume処理
	}
}
////

/* オーバーサンプリング、音量処理用バッファ宣言
 384k用バッファを宣言。中間処理で必要な 192,96,48kHz用バッファは個別に持たず、
 384k用バッファ領域を共有しRAMサイズを節約。x2オーバーサンプリング時、使用済
 ソースデータを完成後のデータで上書きしている。
 topの2ワードは、前回最終データを書き込み、ASRCのOverlap処理用としている
 dsp_buf_top  (0)    -+-------+                               +-------+
                      |Overlap| <- for ASRC                   |       |
 dsp_buf_384k (0/8+2)-+-------+                               +-------+
                      |       |                         x2    |       |
                      |       |                     OverSamp. |       |
                      |       |               x2        +---> |       |
 dsp_buf_192k (4/8+2)_|_ _ _ _|     x2    OverSamp.  ___:___  |  384k |
                      |       | OverSamp.     +---> |       | |       |
 dsp_buf_96k  (6/8+2)_|_ _ _ _|     +--->  ___:___  |  192k | |       |
 dsp_buf_48k  (7/8+2)_|_ _ _ _|  ___:___  |  96k  | |       | |       |
                      |_______| |__48k__| |_______| |_______| |_______|
*/
static int32_t dsp_buf_top[QUEUE_WIDTH +2] = {0};	// 384kHz用 QUEUE_WIDTHと同サイズで準備 +2はASRC処理用Overlap領域
int32_t* const dsp_buf_384k	= &dsp_buf_top[QUEUE_WIDTH * 0 / 8 +2];	// 384kHz用 (領域共有)
int32_t* const dsp_buf_192k	= &dsp_buf_top[QUEUE_WIDTH * 4 / 8 +2];	// 192kHz用 (領域共有)
int32_t* const dsp_buf_96k	= &dsp_buf_top[QUEUE_WIDTH * 6 / 8 +2];	//  96kHz用 (領域共有)
int32_t* const dsp_buf_48k	= &dsp_buf_top[QUEUE_WIDTH * 7 / 8 +2];	//  48kHz用 (領域共有)

// fs(サンプリング周波数)に応じたバッファポインタを返す関数
////多少強引であるが、周波数に応じたケースLEDの発色機能を追加
int32_t* get_dsp_buf_pointer(uint fs){
	switch(fs){
		case 384000:
		case 352800:
			return dsp_buf_384k;
			caseledall(false);
			caseledR(true );	
		case 192000:
		case 176400:
			return dsp_buf_192k;
			caseledall(false);
			caseledR(true );
		case 96000:
		case 88200:
			return dsp_buf_96k;
			caseledall(false);
			caseledG(true );
		case 48000:
		case 44100:
		default:
			return dsp_buf_48k;
			caseledall(false);
			caseledB(true );
	}
}

// 連結ハーフバンドフィルタによるオーバーサンプリング処理
// hbf1~3 の連結数を切り替え、x2 ~ x8 オーバーサンプリングを構成、全fs入力を352.8/384kHzに統一
// 3段 ( 44k1, 48k)->[hbf1]-( 88k2/ 96k)->[hbf2]-(176k4/192k)->[hbf3]-+-(352k8/384k)-->
// 2段 ( 88k2, 96k)---------------------->[hbf1]-(176k4/192k)->[hbf2]-+
// 1段 (176k4,192k)------------------------------(176k4/192k)->[hbf1]-+
// 0段 (352k8,384k)---------------------------------------------------+
//
// len(データ長)はオーバーサンプリング処理回数により2^Nに増加するため、ポインタ渡しとして処理後の長さに書き換える
// 元々はUSBの_as_audio_packet内の処理だったが、I2S側でも使用するため関数化した
void hbf_oversampler(int32_t** buf, uint *p_len, uint fs){
	DEBUG_PIN(PIN_GP12, 1);
	switch(fs){
	  case 384000 :
	  case 352800 :
		// 0段 オーバーサンプリングせずに終了
		break;
	  case 192000 :
	  case 176400 :
		// 1段
		hbf1_x2_oversampler(dsp_buf_192k, dsp_buf_384k, p_len);	DEBUG_PIN(PIN_GP12, 0);
		break;
	  case  96000 :
	  case  88200 :
		// 2段
		hbf1_x2_oversampler(dsp_buf_96k,  dsp_buf_192k, p_len);	DEBUG_PIN(PIN_GP12, 0);
		hbf2_x2_oversampler(dsp_buf_192k, dsp_buf_384k, p_len);	DEBUG_PIN(PIN_GP12, 1);
		break;
	  case  48000 :
	  case  44100 :
	  default     :
		// 3段
		hbf1_x2_oversampler(dsp_buf_48k,  dsp_buf_96k,  p_len);	DEBUG_PIN(PIN_GP12, 0);
		hbf2_x2_oversampler(dsp_buf_96k,  dsp_buf_192k, p_len);	DEBUG_PIN(PIN_GP12, 1);
		hbf3_x2_oversampler(dsp_buf_192k, dsp_buf_384k, p_len);	DEBUG_PIN(PIN_GP12, 0);
		break;
	}
	DEBUG_PIN(PIN_GP12, 0);
	*buf = dsp_buf_384k;
}

/*////

// ASRC 固定小数点精度の定義
#define	ASRC_FRAC_BIT	22
#define ASRC_ALPHA_BIT	 8

// ASRCのリサンプリング位置とバッファ
static uint32_t	asrc_pos = 0;
static int32_t asrc_buf[QUEUE_WIDTH + 2];

 * ASRC処理
 * ASRC:Asynchronous Sampling Rate Converter
 * fpn_delta : 10.22 整数部10bit、小数部22bit

void asrc(int32_t** buf, uint* p_len, uint32_t pitch)
{
	uint len_i = *p_len;	// ASRC入力データ数
	uint len_o = 0;			// ASRC出力データ数
	int32_t* p_i = *buf;		// ASRC入力ポインタ
	int32_t* p_o = asrc_buf; 	// ASRC出力ポインタ

	*buf -= 2;	// ASRC用にBUF先頭をオーバーラップ領域に移動

	// asrc_pos整数部が入力サンプル数未満の場合、処理繰り返し
	while((asrc_pos >> ASRC_FRAC_BIT) < len_i) {
		// ASRCリサンプリング位置計算
		p_i = *buf + 2*(asrc_pos >> ASRC_FRAC_BIT);

		// asrc_posの小数部上位8bitをαブレンド値とする
		interp0->accum[1]
			= (asrc_pos >> (ASRC_FRAC_BIT - ASRC_ALPHA_BIT))&0xff;

		// Ch0処理 入力データ2点間のαブレンド値をバッファに出力 
		interp0->base[0] = p_i[0];	// データb0
		interp0->base[1] = p_i[2];	// データb1
		*p_o++ =interp0->peek[1];	// (1-α)*b0 +α*b1

		// Ch1処理 入力データ2点間のαブレンド値をバッファに出力 
		interp0->base[0] = p_i[1];	// データb0
		interp0->base[1] = p_i[3];	// データb1
		*p_o++ =interp0->peek[1];	// (1-α)*b0 +α*b1

		len_o ++;			// ASRCデータ数更新
		asrc_pos += pitch;	// ASRCポジション更新
	}

	// 次回ASRCポジションを更新 処理済みのデータ長分を減算
	asrc_pos -= (len_i << ASRC_FRAC_BIT);

	// 
	p_i = *buf;
	p_i[0] = p_i[len_i * 2];
	p_i[1] = p_i[len_i * 2 + 1];

	// ASRCにより変化したデータ長とバッファポインタを返却
	*p_len = len_o;
	*buf = asrc_buf;
}

void asrc_reset(void){
	// asrc用オーバーラップ領域のクリア
	dsp_buf_top[0] = 0;
	dsp_buf_top[1] = 0;
	// asrcポジションのクリア
	asrc_pos = 0;
}
*////

void dsp_reset(void){
	hbf_oversampler_reset();
////	asrc_reset();
}

void dsp_init(void){
	interp1_hw_clamp_init();
	interp0_blender_init();
	dsp_reset();
}