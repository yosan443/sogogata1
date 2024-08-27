/**
 * @file bsp.c
 * @author geachlab, Yasushi MARUISHI
 * @brief RP2040, Pico, pico_1bit_dac, pico_diy_dac_v2 ボード依存の関数類
 * @version 0.01
 * @date 2023-02-21
 * @note Pico基板, DAC基板依存ソフトウェア
 */

#include "pico/stdlib.h"
#include "bsp.h"
//#include "pins.h"

// gpio pinの初期化
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


// 全ピン初期化 
// HAT関連端子はラズパイ未接続にFloatとならぬよう Pullupを入れる
void all_gpio_init(void){
	////////////GPIO_NUM         I/O      Lv PU PD  Strength                 Slew rate
	// UART関連 uart_init内で再設定される 
	gpio_config(PIN_UART0_TX   , GPIO_OUT, 1, 0, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);
	gpio_config(PIN_UART0_RX   , GPIO_IN , 0, 1, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);

	// I2C関連
	gpio_config(PIN_I2C0_SDA   , GPIO_IN , 0, 1, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);
	gpio_config(PIN_I2C0_SCL   , GPIO_IN , 0, 1, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);
	gpio_config(PIN_I2C1_SDA   , GPIO_IN , 0, 1, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);
	gpio_config(PIN_I2C1_SCL   , GPIO_IN , 0, 1, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);

	// DIP SW関連 4bit分
  for(uint i = PIN_DIP_0; i  < (PIN_DIP_0 + PIN_DIP_BIT_WIDTH); i++)
	gpio_config(i              , GPIO_IN , 0, 1, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);

	// Debug/RFU関連
	gpio_config(PIN_GP10       , GPIO_OUT, 0, 0, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);
	gpio_config(PIN_GP11       , GPIO_OUT, 0, 0, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);
	gpio_config(PIN_GP12       , GPIO_OUT, 0, 0, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);
	gpio_config(PIN_GP13       , GPIO_OUT, 0, 0, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);
	gpio_config(PIN_GP18       , GPIO_OUT, 0, 0, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);
	gpio_config(PIN_GP26       , GPIO_OUT, 0, 0, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);
	gpio_config(PIN_GP27       , GPIO_OUT, 0, 0, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);

	// PDM出力関連(GPIO14~17)
	// pdm_output.c(Core1)側で実施

	// I2S関連
	// LRCK, BCK は入力始まり(target)なのでInput&PU、DATA_OUTは当面未使用のためInput&PUとしている
	gpio_config(PIN_I2S_SDO    , GPIO_IN , 0, 0, 1, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);
	gpio_config(PIN_I2S_SDI    , GPIO_IN , 0, 0, 1, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);
	gpio_config(PIN_I2S_LRCK   , GPIO_IN , 0, 0, 1, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);
	gpio_config(PIN_I2S_BCK    , GPIO_IN , 0, 0, 1, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);

	// Pico DCDC関係 U2:RT6150B を PWM MODE(1) で動作させDCDC固有ノイズを低減する
	gpio_config(PIN_DCDC_PWM   , GPIO_OUT, 1, 0, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);
//	gpio_config(PIN_DCDC_PWM   , GPIO_OUT, 0, 0, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);

	// Pico 制御関係
	gpio_config(PIN_VBUS_DETECT, GPIO_IN , 0, 0, 1, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);
	gpio_config(PIN_PICO_LED   , GPIO_OUT, 0, 0, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);

	// PDM 制御関連
	gpio_config(PIN_FS48       , GPIO_OUT, 0, 0, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_FAST);

	// その他
	gpio_config(PIN_VSYS_LEVEL , GPIO_IN , 0, 0, 1, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_FAST);


}

// DAC動作状態(4bit)をGPIO出力
void change_dac_state_gpio(void)
{
#if 0
	static bool first_time = true;
	if (first_time){
		for (uint i = 0; i <= 3; i ++){
			gpio_init(PIN_DAC_STATE_GPIO + i);
			gpio_set_dir(PIN_DAC_STATE_GPIO + i, GPIO_OUT);
		}
		first_time = false;
	}
	uint gpio_pat;
	switch(audio_state.fs){
		case 192000	: gpio_pat = 0xa; break;
		case 176400	: gpio_pat = 0x8; break;
		case  96000	: gpio_pat = 0x6; break;
		case  88200	: gpio_pat = 0x4; break;
		case  48000	: gpio_pat = 0x2; break;
		case  44100	:
		default		: gpio_pat = 0x0;
	}
	switch(audio_state.bit_depth){
		case     24 : gpio_pat |= 0x1; break;
		case     16 :
		default     : gpio_pat |= 0x0;
	}
	gpio_put_masked(0xf<<PIN_DAC_STATE_GPIO, (gpio_pat &0xf)<<PIN_DAC_STATE_GPIO);
#endif
}


// Pico USB VBUS(5V) input Status
// return true:VBUS=5V,false:VBUS=0V or Under Voltage
bool get_pico_usb_vbus_status(void){
//	gpio_config(PIN_VBUS_DETECT, GPIO_IN , 0, 0, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);
	return gpio_get(PIN_VBUS_DETECT);
}

// Pico Onboard LED Control
// true:LED On,false:LED Off
void set_pico_onboard_led(bool value){
//	gpio_config(PIN_PICO_LED,    GPIO_OUT, value, 0, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);
	gpio_put(PIN_PICO_LED, value);
}

// Pico Onboard LED Control
// true:LED On,false:LED Off
void set_dac_onboard_led(bool value){
//	gpio_config(PIN_PICO_LED,    GPIO_OUT, value, 0, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_SLOW);
	gpio_put(PIN_DAC_LED, value);
}


// DipSW Init. & DipSW Reader
uint get_dip(void){
    static bool first = 1;
    if (first){
        first = false;
        gpio_init_mask(PIN_DIP_MASK);
        gpio_set_dir_in_masked(PIN_DIP_MASK);
        for(uint32_t i = 0; i < PIN_DIP_BIT_WIDTH; i++)
            gpio_pull_up(PIN_DIP_0 +i);
    }
    return (gpio_get_all() & PIN_DIP_MASK) >> PIN_DIP_0;
}


// PDM fs系列切替処理 _HR2までは pdm_output.c に実装
// 旧set_pdm_fs_gpio(uint fs)を bool group_48k対応にしたもの
void set_dac_fs_group_48k(bool group_48k){
//	gpio_config(PIN_FS48       , GPIO_OUT, 0, 0, 0, GPIO_DRIVE_STRENGTH_2MA, GPIO_SLEW_RATE_FAST);
	gpio_put(PIN_FS48, group_48k);	// 周波数系列軸(44.1k/48k)に合わせた設定を行う
};
