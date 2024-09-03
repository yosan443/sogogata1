////2024年8月27日　菅原嘉美
////このファイルの基となったbsp.cはラズパイPicoDACの主に専用基板用の制御プログラムであったが、今回の機器にはその基板は用いていない。そのため、基本動作に必要なプログラムと今回の機器の仕様に依存する部分のプログラムのみを新たに記述した。

#include "pico/stdlib.h"
#include "pins.h"
//#include "pins.h"

//// ピンの初期化。ここはbsp.cと同様。
void gpio_config(
	uint gp,	bool output,	bool value,	bool pullup,	bool pulldown,
	enum gpio_drive_strength drive,	enum gpio_slew_rate slew
){
	gpio_init(gp);
	gpio_set_dir(gp, output);
	gpio_set_pulls(gp, pullup, pulldown);
	if(output == true){
		gpio_set_drive_strength(gp, drive);
		gpio_set_slew_rate(gp, slew);
		gpio_put(gp, value);
	}
}

//// システム関連ピン初期化 
void all_gpio_init(void){

	//// 内臓DCDCのモードを常にPWMモードに設定
	gpio_config(GP23 , GPIO_OUT, 1, 0, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);


////システムの状態判別に有用であったためbsp.cから流用。ボード上のLEDの発光を定義。
void set_pico_onboard_led(bool value){
	gpio_put(PIN_PICO_LED, value);
}

void caseledall(bool value){
	gpio_put(CASELEDR, value);
	gpio_put(CASELEDG, value);
	gpio_put(CASELEDB, value);
}
	
void caseledR(bool value){
	gpio_put(CASELEDR, value);
}

void caseledG(bool value){
	gpio_put(CASELEDG, value);
}

void caseledB(bool value){
	gpio_put(CASELEDB, value);
}

////↓この部分はbsp.cから抜き出し。
// PDM fs系列切替処理 _HR2までは pdm_output.c に実装
// 旧set_pdm_fs_gpio(uint fs)を bool group_48k対応にしたもの
void set_dac_fs_group_48k(bool group_48k){
//	gpio_config(PIN_FS48       , GPIO_OUT, 0, 0, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_FAST);
	gpio_put(PIN_FS48, group_48k);	// 周波数系列軸(44.1k/48k)に合わせた設定を行う
};
