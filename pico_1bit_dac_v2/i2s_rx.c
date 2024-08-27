/**
 * @file i2s_rx.c
 * @author geachlab, Yasushi MARUISHI
 * @brief pico_1bit_dac_v2用 i2s処理(pico_1bit_dac_HR2より派生)
 * @version 0.01
 * @date 2023.02.21
 * @note HAT DAC I2S通信部分
 */

// I2S受信デバッグ時に1とする
#if 0
#define DEBUG_PIN(x, y) gpio_put((x), (y))
#else
#define DEBUG_PIN(x, y) /*処理なし*/
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/dma.h"

#include "audio_state.h"
#include "bsp.h"
#include "i2s_rx.pio.h"
#include "dsp.h"
#include "simple_queue.h"
#include "pdm_output.h"

// マクロ定義 入力 x が typ に対し偏差ppm以内のとき true
// (1ppm = 10^-6 ≒ 2^-20 = 0.953ppm)
// 入力 x が typ に対し偏差ppm以内のとき true (1ppm = 10^-6 ≒ 2^-20 = 0.953ppm)
#define IN_RANGE_PPM(x,typ,ppm) ((x <= (typ+(typ*ppm>>20))) && (x >= (typ-(typ*ppm>>20))))
// 入力 x を min, max範囲に収める if(x>max) x=max; else if(x<min) x=min;)
#define CONSTRAIN(x,min,max) (x = (x > max) ? max : (x < min) ? min : x)

// 受信サンプル数関連定義
#define I2S_RX_SAMPLES	((uint32_t)48)	//データ受信数(片Ch分)
#define I2S_RX_CHANNELS	2				//2Ch

// PIO/SM/関連定数設定
const PIO PIO_I2S = pio1;	// pio0は既存のCore1 PDMで使用済みのため pio1を使用
enum pio_sm_num {
	SM_I2S_RX = 0,			// sm0
	SM_LRCK_COUNTER,		// sm1
	SM_BCK_COUNTER,			// sm2
	SM_LONG_LRCK_COUNTER	// sm3
};

// lrck/bckカウント数はデータ受信数の1/2(ナイキスト)以下とする
const uint LRCK_COUNTER_N = I2S_RX_SAMPLES / 2; //データ数の1/2(ナイキスト)
const uint BCK_LRCK_N_RATIO = 32; //1LRCKに対するBCK計測数の比
const uint BCK_COUNTER_N  = LRCK_COUNTER_N * BCK_LRCK_N_RATIO;
const uint long_lrck_counter_n = I2S_RX_SAMPLES * 1000; // 1sec @ 48kHz

extern audio_state_t audio_state;

// I2Sデータ(クロック)有効・無効判定＆audio_state 変数群更新
// I2Sクロックが連続的に正規なことを判断基準とする
static inline bool i2s_data_valid(void){
	static int lrck_count_o = 0, bck_count_o = 0;

	// LRCK,BCK 新カウント値取得
	int lrck_count = get_clock_width_counter_w(SM_LRCK_COUNTER);
	int bck_count  = get_clock_width_counter_w(SM_BCK_COUNTER);
	DEBUG_PIN(GP11,1);

	// 新旧カウント差が±1を超えていたらカウント値変化と判定
	int delta_lrck_count = lrck_count - lrck_count_o;
	int delta_bck_count  = bck_count  - bck_count_o;
	bool count_changed
		=  (delta_lrck_count > 1)||(delta_lrck_count < -1)
		|| (delta_bck_count  > 1)||(delta_bck_count  < -1)
		|| (!audio_state.data_valid);

	// 新旧カウント値が不変ならば高速化のため判定処理をスキップ
	if(!count_changed)
	{
		audio_state.format_updated = false;
	}
	// カウント値が0の場合は計算不能のため非正規として終了
	else if((lrck_count == 0)||(bck_count == 0))
	{
		audio_state.format_updated = false;
		audio_state.data_valid = false;
	}
	// BCK/LRCKカウント値変化時・前回非正規データ時の処理
	else
	{
		// 旧カウンタ値更新
		lrck_count_o = lrck_count;
		bck_count_o  = bck_count;

		// カウント値からLRCK周波数=サンプリング周波数fsを得る
		uint fs=((CLK_SYS/2)*LRCK_COUNTER_N)/(uint)lrck_count;

		// 規格周波数±250ppm範囲内を正規周波数値に変換、範囲外を0とする
		if		(IN_RANGE_PPM(fs,384000, 250)) fs =384000;
		else if	(IN_RANGE_PPM(fs,352800, 250)) fs =352800;
		else if	(IN_RANGE_PPM(fs,192000, 250)) fs =192000;
		else if	(IN_RANGE_PPM(fs,176400, 250)) fs =176400;
		else if	(IN_RANGE_PPM(fs, 96000, 250)) fs = 96000;
		else if	(IN_RANGE_PPM(fs, 88200, 250)) fs = 88200;
		else if	(IN_RANGE_PPM(fs, 48000, 250)) fs = 48000;
		else if	(IN_RANGE_PPM(fs, 44100, 250)) fs = 44100;
		else /* unknown format frequency */    fs =     0;

		// LRCK・BCKカウント比(1LRCK中のBCK数)からビット深度を算出
		uint bit_depth
		//	= BCK_LRCK_N_RATIO * lrck_count / bck_count/2+0.5
			=(BCK_LRCK_N_RATIO * lrck_count / bck_count + 1) >> 1;
		// ビット深度32/24/16を正規ビット深度と判定、それ以外を0とする
		switch(bit_depth){
			case 32 :
			case 24 :
			case 16 : break; // Standard bit depth
			default : bit_depth = 0; break; // unknown bit depth
		}

		// 新旧周波数・ビット深度が不変かつ0以外を有効データと判定
		// クロック変化点はデータ破損の可能性があるため無効データとする
		bool data_valid
			=  (fs != 0)				// fs(フォーマット周波数)が正規
			&& (bit_depth != 0)			// ビット深度が正規
			&& (fs == audio_state.fs)	// 新旧fsが一致
			&& (bit_depth == audio_state.bit_depth);	//新旧ビット長が一致

		// 無効データ→有効データ変化点でデータ更新フラグを立てる
		if(data_valid && !audio_state.data_valid){
			audio_state.format_updated = true;
		}

		// audio_state値更新
		audio_state.fs            = fs;
		audio_state.bit_depth     = bit_depth;
		audio_state.osr           = get_osr(fs);
		audio_state.group_48k_src = get_group_48k(fs);
//		audio_state.group_48k_dac = 0;	// 44k固定
		audio_state.group_48k_dac = !audio_state.group_48k_src; // 異なる周波数系としてうねりを拡散
		audio_state.data_valid    = data_valid;
	}
	DEBUG_PIN(GP11,0);
	return audio_state.data_valid;
}

// i2sバッファ→ローカルバッファコピー処理
// I2Sビット長に応じた整形処理(32/24/16bit右詰→24bit固定長)も行う
static inline void i2s_buf_copy(uint32_t* p_i, int32_t* p_o, uint* len, uint bit_depth){
	uint sample_num = *len;	
	switch(bit_depth){
	  case 32 :	// 32bit->24bit幅ステレオコピー 下位8bitは破棄される
		while(sample_num--){
			*p_o++ = ((int32_t)*p_i++) >> 8;
			*p_o++ = ((int32_t)*p_i++) >> 8;
		//	*p_o++ = sample_num << 16;		// 周波数FeedBack追従試験用。LChに1kHz Sin波を入力し、LRのリサージュ波形を観測する。
		}
		break;
	  case 24 :	// 24bit->24bit幅ステレオコピー 要Sign Bit拡張
		while(sample_num--){
			*p_o++ = ((int32_t)(*p_i++ << 8)) >> 8;
			*p_o++ = ((int32_t)(*p_i++ << 8)) >> 8;
		}
		break;
	  case 16 :	// 16bit->24bit幅ステレオコピー 要Sign Bit拡張
		while(sample_num--){
			*p_o++ = ((int32_t)(*p_i++ << 16)) >> 8;
			*p_o++ = ((int32_t)(*p_i++ << 16)) >> 8;
		}
		break;
	  default :
		//不明なビットフォーマットではコピーを行わずlen=0とする
		*len = 0;
		break;
	}
}

/**
 * DMA関連設定
 * 資料 : 
 * https://raspberrypi.github.io/pico-sdk-doxygen/group__hardware__dma.html
 * https://raspberrypi.github.io/pico-sdk-doxygen/group__channel__config.html
 */
#define I2S_RX_DMA_CH0	0
#define I2S_RX_DMA_CH1	1
const uint32_t dma_receive_size = I2S_RX_SAMPLES * I2S_RX_CHANNELS;
uint32_t i2s_rx_buf[2][I2S_RX_SAMPLES + 2][I2S_RX_CHANNELS];

// I2Sデータ受信処理 割り込みハンドラから呼ばれる
static inline void i2s_rx_data_received(uint32_t* samples, uint nsamples)
{
	// 前回データ処理中の場合は処理中止()
	if(audio_state.data_received) return;

	DEBUG_PIN(PIN_GP11, 0);

	// LRCK/BCKの正規フォーマット判定＆audio_state変数更新
	// 非正規なら同時刻のI2S受信データは無効データとして読み取らず終了
	if(!i2s_data_valid()){
		audio_state.data_received = false;
		return;
	}
	DEBUG_PIN(PIN_GP11, 1);

	// 入力fsに応じたdspバッファポインタを取得
	audio_state.dsp_buf = get_dsp_buf_pointer(audio_state.fs);
	audio_state.len = nsamples; 

	// I2Sデータコピー
	i2s_buf_copy(samples, audio_state.dsp_buf, &audio_state.len, audio_state.bit_depth);
	audio_state.data_received = true;
	DEBUG_PIN(PIN_GP11, 0);
}


void dma0_handler(void)
{
	uint plane;
	if(dma_hw->intr & (1u << I2S_RX_DMA_CH0)){
		dma_hw->ints0 = 1u << I2S_RX_DMA_CH0;
		dma_channel_set_write_addr(I2S_RX_DMA_CH0, i2s_rx_buf[0][0], false);
		plane = 0;
	} else if(dma_hw->intr & (1u << I2S_RX_DMA_CH1)){
		dma_hw->ints0 = 1u << I2S_RX_DMA_CH1;
		dma_channel_set_write_addr(I2S_RX_DMA_CH1, i2s_rx_buf[1][0], false);
		plane = 1;
	} else return;
	DEBUG_PIN(PIN_GP10, plane);	// 利用プレーンをGPIOでモニタ 
	i2s_rx_data_received(i2s_rx_buf[plane][0], I2S_RX_SAMPLES);
}

void i2s_dma_init(PIO pio, uint sm)
{
	// 全DMA IRQをクリアしておく
	printf("dma_hw->ints0 = %08x\n",dma_hw->ints0);
	dma_hw->ints0 = 0xffff;

	// I2S RX DMA CH1のセットアップ
	dma_channel_config rx_dma1_config = dma_channel_get_default_config(I2S_RX_DMA_CH1);
	channel_config_set_read_increment(&rx_dma1_config, false);
	channel_config_set_write_increment(&rx_dma1_config, true);
	channel_config_set_dreq(&rx_dma1_config, pio_get_dreq(pio, sm, false));
	channel_config_set_chain_to(&rx_dma1_config, I2S_RX_DMA_CH0);

	dma_channel_set_irq0_enabled(I2S_RX_DMA_CH1, true);

	dma_channel_configure(
		I2S_RX_DMA_CH1,	// DMAチャネル
		&rx_dma1_config,		// DMA設定構造体ポインタ
		i2s_rx_buf[1],		// ライトするアドレス
		&pio->rxf[sm],			// リードするアドレス
		dma_receive_size,		// 転送量(単位は32bit/wordのword数)
		false					// 即時転送開始=true
	);

	// I2S RX DMA CH0のセットアップ
	dma_channel_config rx_dma0_config = dma_channel_get_default_config(I2S_RX_DMA_CH0);
	channel_config_set_read_increment(&rx_dma0_config, false);
	channel_config_set_write_increment(&rx_dma0_config, true);
	channel_config_set_dreq(&rx_dma0_config, pio_get_dreq(pio, sm, false));
	channel_config_set_chain_to(&rx_dma0_config, I2S_RX_DMA_CH1);

	dma_channel_set_irq0_enabled(I2S_RX_DMA_CH0, true);
	irq_set_exclusive_handler(DMA_IRQ_0, dma0_handler);
	irq_set_enabled(DMA_IRQ_0, true);

	dma_channel_configure(
		I2S_RX_DMA_CH0,	// DMAチャネル
		&rx_dma0_config,		// DMA設定構造体ポインタ
		i2s_rx_buf[0],		// ライトするアドレス
		&pio->rxf[sm],			// リードするアドレス
		dma_receive_size,		// 転送量(単位は32bit/wordのword数)
		true					// 即時転送開始=true
	);
}

// i2s初期化
void i2s_init(void){

	// audio_state 初期値更新
	audio_state.data_valid = false;	
	audio_state.format_updated = false;	
	audio_state.data_received = false;	

	// dma初期化
	i2s_dma_init(PIO_I2S, SM_I2S_RX);

	// pio初期化
	i2s_rx_target_program_init(PIO_I2S, PIN_I2S_SDI);
	clock_width_counter_program_init(PIO_I2S, PIN_I2S_SDI);

	// クロックカウンタ群初期化
	set_clock_width_counter_n(SM_LRCK_COUNTER, LRCK_COUNTER_N);
	set_clock_width_counter_n(SM_BCK_COUNTER,  BCK_COUNTER_N);
	set_clock_width_counter_n(SM_LONG_LRCK_COUNTER,  long_lrck_counter_n);

	puts("I2S init done.");
}


#define P_44TO44 ((uint32_t)((uint64_t)(1u << 22) * 44100 * 74 * 64 / CLK_SYS))	// 44k->44k : 1.000275862...
#define P_44TO48 ((uint32_t)((uint64_t)(1u << 22) * 44100 * 68 * 64 / CLK_SYS))	// 44k->48k : 0.919172413...
#define P_48TO44 ((uint32_t)((uint64_t)(1u << 22) * 48000 * 74 * 64 / CLK_SYS))	// 48k->44k : 1.088735632...
#define P_48TO48 ((uint32_t)((uint64_t)(1u << 22) * 48000 * 68 * 64 / CLK_SYS))	// 48k->48k : 1.000459770... 
#define P_TOL    ((uint32_t)(1u<<(22-13)))	// (1<<(22-13)/(1<<22) = 1 / 8192 ≒ 125ppm 
const uint32_t pitch_typ[4] = {P_44TO44,        P_44TO48,        P_48TO44,        P_48TO48};
const uint32_t pitch_max[4] = {P_44TO44 +P_TOL, P_44TO48 +P_TOL, P_48TO44 +P_TOL, P_48TO48 +P_TOL};
const uint32_t pitch_min[4] = {P_44TO44 -P_TOL, P_44TO48 -P_TOL, P_48TO44 -P_TOL, P_48TO48 -P_TOL};

const uint64_t pitch_num[2] = { (uint64_t)(8 * 74 * 4 * long_lrck_counter_n) * (1u<<22),
								(uint64_t)(8 * 68 * 4 * long_lrck_counter_n) * (1u<<22)};

// 長周期カウントの標準値 
const int count_typ[2] = { (CLK_SYS / 44100 * long_lrck_counter_n / 2),
						   (CLK_SYS / 48000 * long_lrck_counter_n / 2)};



// ASRC用変換ピッチ演算
// asrc(asynchronous sampling rate converter)のリサンプリングピッチ取得
bool asrc_pitch_update(void){
//	static int32_t pitch;	// 内部保持用ピッチ
	static int count_o = 0;

	// 変換モード(0:44k->44k,1:44k->48k,2:48k->44k,3:48k->48k)
	uint pitch_mode = ((uint)audio_state.group_48k_src << 1)
					| ((uint)audio_state.group_48k_dac);

	// フォーマット変更時はASRCピッチが変わるため、標準(Typ)値を設定
	if(audio_state.format_updated){
		audio_state.asrc_pitch = pitch_typ[pitch_mode];
		audio_state.format_updated = false;

		// 長周期LRCKカウンタ設定
		// カウント数はOSR(OverSamplingRate)を乗じて周期を正規化
		set_clock_width_counter_n(
			SM_LONG_LRCK_COUNTER,
			long_lrck_counter_n * audio_state.osr);
		// 旧カウント値には標準値を代入
		count_o = count_typ[audio_state.group_48k_src];
		return true;
	}

	// 長周期LRCKカウンタが fifo empty = 更新なしの場合 false を返却
	if(pio_sm_is_rx_fifo_empty(PIO_I2S, SM_LONG_LRCK_COUNTER))
		return false;
	// 長周期LRCKカウンタ値取得
	int count = get_clock_width_counter_w(SM_LONG_LRCK_COUNTER);

	// 新旧カウント値の差が1000ppm以上の場合は異常値として終了
	int delta = count - count_o;
	if(   (delta > +(CLK_SYS / 2 * 0.001))
		||(delta < -(CLK_SYS / 2 * 0.001))) return false;
	count_o = count;

	audio_state.count_long = count;
//	DEBUG_PIN(GP11,1);
	uint32_t pitch = (uint32_t)(pitch_num[pitch_mode & 1] / (uint64_t)count);
//	uint32_t pitch = (uint32_t)((float)pitch_num[pitch_mode & 1] / (float)count); 
//	DEBUG_PIN(GP11,0);

	// QUEUEの水準で適応フィードバックをかける
//	int error = get_queue_length() - QUEUE_TARGET_WATERLEVEL;	// USB用の水準
	int error = get_queue_length() - QUEUE_PLAY_THR;	// 再生開始水準
	error = CONSTRAIN(error, -5, +5);	// ±5ppmのエラーフィードバックに相当 
	pitch += error * 4;	// 4/(1u<22) = 1/(1u<<20) ≒ 1ppm /step

	// 範囲制限
	pitch = CONSTRAIN(pitch, pitch_min[pitch_mode], pitch_max[pitch_mode]);
	audio_state.asrc_pitch = pitch; 
	return true;

/*  適応Feedback
    キューサンプル数がターゲット水位に接近すると、フィードバック量が小さくなるよう制御する。
    これによりキューサンプル数がターゲット水位で安定し、目標周波数に平衡する
            error (feedback)
    |+5_____  A
    |       \ :
    |        \:
  0 +---------+----------> get_queue_length()
    |         :\
    |         : \_____-5
    |         :
	0   QUEUE_PLAY_THR
*/
}