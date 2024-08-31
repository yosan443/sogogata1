////2024年8月27日　菅原嘉美
////元のbsp.hには今回の機器には不要なものが多いと判断したため、それを参考に新たに作成。

#ifndef _BSP_H_
#define _BSP_H_

void all_gpio_init(void);
void set_pico_onboard_led(bool value);
void set_dac_fs_group_48k(bool group_48k);

//// システムクロック周波数指定 
#define CLK_SYS	((uint32_t)208800000)
#define DEFAULT_FS      44100           // Source Sampling Frequency[Hz]
#define DEFAULT_BIT_DEPTH  16
#define N_CH            2               // Number of Audio Channel
#define DS_BITSHIFT     7               // DeltaSigma 演算前ビットシフト量 Ex. (24bit data)<<DS_BITSHIFT

#define PIN_TIME_MEASURE    12                  //// Core1 PDM処理時間計測用
#define PIN_PIOT_MEASURE    13                  //// Core1 PIO処理時間計測用
#define PIN_OUTPUT_RP       14                  //// RCh
#define PIN_OUTPUT_LP       16                  //// LCh
#define PIN_PICO_LED        25                  //// rp2040搭載ボード上のLED
#define CASELEDR            26                  ////ケースRGBLEDの赤色発光用ピン
#define CASELEDG            27                  ////ケースRGBLEDの緑色発光用ピン
#define CASELEDB            28                  ////ケースRGBLEDの青色発光用ピン

////使用しないピンもあるがbsp.hにならい記述
#define GP00	0
#define GP01	1
#define GP02	2
#define GP03	3
#define GP04	4
#define GP05	5
#define GP06	6
#define GP07	7
#define GP08	8
#define GP09	9
//#define GP10	10
//#define GP11	11
//#define GP12	12
//#define GP13	13
#define GP14	14
#define GP15	15
#define GP16	16
#define GP17	17
//#define GP18	18
#define GP19	19
#define GP20	20
#define GP21	21
#define GP22	22
#define GP23	23
#define GP24	24
#define GP25	25
//#define GP26	26
//#define GP27	27
#define GP28	28
#define GP29	29

#endif
