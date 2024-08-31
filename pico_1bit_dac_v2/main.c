/**
 * @file main.c
 * @brief pico_1bit_dac_v2用 (pico_1bit_dac_HR2より派生)
 * @author geachlab, Yasushi MARUISHI
 * @version 0.01
 * @date 2023.02.21
 * @note USB DAC / HAT DAC両対応版
 *       I2S受信」機能追加に伴い、ファイル構造を整理
 * 旧:  pico_1bit_dac.c 初期化,USB受信/音量/DSP/再生処理,misc
 *      oversampler.c/h 前段x2~x8オーバーサンプリング
 *      pin.h           GPIOピン命名・定義
 * 新:  main.c          初期化, USB/I2S処理ブランチ, USB/I2S共通再生処理
 *      usb_audio.c/h   USB 初期化, USB 受信処理
 *      i2s.c/h         I2S 初期化, I2S 受信処理
 *      dsp.c/h         音量, 前段x1~x8オーバーサンプリング, ASRC
 *      bsp.c/h         ボード依存処理・GPIO定義・初期化
 * 継承:simple_queue.c/h Core0->Core1 PCMデータキュー管理
 *      pdm_output.c/h  後段x8オーバーサンプリング、ΔΣ、PWM出力
 */

// デバッグ時に1とする
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
#include "hardware/vreg.h"

#include "audio_state.h"
#include "bsp.h"
#include "usb_audio.h"
////#include "i2s_rx.h"
#include "dsp.h"
#include "simple_queue.h"
#include "pdm_output.h"

audio_state_t audio_state;

int main(void) {
	vreg_set_voltage(VREG_VOLTAGE_1_30);	// Core電圧Up 1.1V->1.3V メリット:S/Nが約3dB改善する デメリット:消費電力増(未測定)
	//  PDM動作に最適なCPU周波数の設定
    set_sys_clock_khz(CLK_SYS/1000, true);    //  208M8/48k/64 = 67.968->68, 208M8/44k1/64 = 73.979->74 x1.57 Overclock

	all_gpio_init();
    stdio_uart_init();
//	uart_init(uart0, 1500000);	// 開発用 Baudを高速にしておき、I2S DMAの競合を回避する
	puts("pico_1bit_dac_v2");

    queue_init();
	dsp_init();
	puts("USB DAC MODE"); ////
	audio_state.source = FROM_USB; ////

	// core1(x8OverSampling~ΔΣ~pdm出力)起動
	multicore_launch_core1(pdm_output);

	// usb_audio/i2s_rx 受信ループ
	while(1){
		// usb/i2s irq処理待ち
		__wfi();

		// オーディオフォーマット更新時の処理
		if(audio_state.format_updated) {
			dsp_reset();	// dsp処理内のフィルタ残存データ破棄
			set_dac_fs_group_48k(audio_state.group_48k_dac);	// DAC fs変更
			printf("Format Updated:%6dHz/%2dbit\n", audio_state.fs, audio_state.bit_depth);
		}

		// オーディオデータ受信時のdsp処理
		if(audio_state.data_received) {
			DEBUG_PIN(PIN_GP13, 1);

			int32_t* dsp_buf = audio_state.dsp_buf;
			uint len = audio_state.len; 

			// 音量処理 現状はUSBソースのみ処理 ////USBのみのため条件分岐は不要
////			if(audio_state.source == FROM_USB) {
////				volume(dsp_buf, len, audio_state.vol_mul, audio_state.vol_shift);
////			}
			volume(dsp_buf, len, audio_state.vol_mul, audio_state.vol_shift);
			DEBUG_PIN(PIN_GP13, 0);

			// 連結ハーフバンドフィルタによる周波数適応オーバーサンプリング処理
			hbf_oversampler(&dsp_buf, &len, audio_state.fs);
			DEBUG_PIN(PIN_GP13, 1);

			// キューオーバーフロー救済処置 オーバーフロー水位でデータから1サンプルを間引く
			uint queue_length = get_queue_length();
			if( queue_length >= QUEUE_DEPTH - 1){
				len--;
			}

			// ASRC処理 I2S_TARGETソースのみ処理 ////I2sソース無効化に伴いASRC処理も全面的に無効化
////			if(audio_state.source == FROM_I2S_TARGET) {
////				if(asrc_pitch_update()){
//					printf("%2d %2d %3d %9.7f %6d\n", queue_length, audio_state.bit_depth, audio_state.fs/1000, (float)audio_state.asrc_pitch/(float)(1<<22), (int32_t)((int64_t)104400000*48000/audio_state.count_long));
////				}
////				asrc(&dsp_buf, &len, audio_state.asrc_pitch);
////			}

			// オーバーサンプリング後のデータをキューに積む
			enqueue(dsp_buf, len);
			DEBUG_PIN(PIN_GP13, 0);

			// 受信データ処理完了
			audio_state.data_received = false;
		}
		audio_state.format_updated = false;
	}
}