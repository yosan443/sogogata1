/**
 * @file audio_state.h
 * @author geachlab, Yasushi MARUISHI
 * @brief USB/I2S受信・再生パラメタ共有構造体定義
 * @version 0.01
 * @date 2023-02-21
 * @note 原型はusb_sound_cardの同名構造体
 *       I2S/USB両対応 mainで宣言、.fs(再生周波数)などを周辺ファイルと共有する
 */
#ifndef _AUDIO_STATE_H_
#define _AUDIO_STATE_H_

enum dac_source {
	FROM_USB = 0,
	FROM_I2S_TARGET,
	FROM_I2S_CONTROLLER
};

typedef struct {
	uint fs;				// 入力サンプリング周波数 = lrck 入力周波数
	uint bit_depth;			// ビット深度
	uint osr;				// オーバサンプリングレート ex. 384k=8x48k;osr=8
	bool group_48k_src;		// 48kHz系列フラグ(ソース) 
	bool group_48k_dac;		// 48kHz系列フラグ(DAC) 
	bool data_valid;		// 正規データフラグ
	bool format_updated;	// フォーマットアップデートフラグ	
	bool data_received;		// 正規データ受信フラグ
	uint32_t asrc_pitch;	// ASRC再生ピッチ 22bit固定小数点
	uint32_t count_long;	// デバッグ用 長周期LRCKカウンタ値
	int16_t volume;			// volume(-127.996 ~ +127.996dB)
	int16_t vol_mul;		// volume処理 乗算量(6dB幅)
	uint32_t vol_shift;		// volume処理 ビットシフト量
	uint32_t *dsp_buf;		// dsp_buf処理ポインタ
	uint len;				// dsp_buf上のサンプル長
	uint source;			// 入力ソース (enum dac_source形式)
	bool mute;
} audio_state_t;

#endif
