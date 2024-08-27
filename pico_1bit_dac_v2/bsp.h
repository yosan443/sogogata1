#ifndef _BSP_H_
#define _BSP_H_

void all_gpio_init(void);
void set_pico_onboard_led(bool value);
bool get_pico_usb_vbus_status(void);
void set_dac_fs_group_48k(bool group_48k);
uint get_dip(void);

// システムクロック周波数指定 
#define CLK_SYS	((uint32_t)208800000)
#define DEFAULT_FS      44100           // Source Sampling Frequency[Hz]
#define DEFAULT_BIT_DEPTH  16
#define N_CH            2               // Number of Audio Channel
#define DS_BITSHIFT     7               // DeltaSigma 演算前ビットシフト量 Ex. (24bit data)<<DS_BITSHIFT

// ピン名-GPIO定義
#define PIN_UART0_TX         0
#define PIN_UART0_RX         1
#define PIN_I2C0_SDA         2                  // Primary
#define PIN_I2C0_SCL         3                  // Primary
#define PIN_I2C1_SDA         4                  // Secondary
#define PIN_I2C1_SCL         5                  // Secondary
#define PIN_DIP_0            6                  // Test DipSW start pin
#define PIN_DIP_MASK        (0xF << PIN_DIP_0)  // Test DipSW Mask position
#define PIN_DIP_BIT_WIDTH    4                  // Test DipSW Bit width
#define PIN_GP10            10                  // Debug / RFU
#define PIN_GP11            11                  // Debug / RFU
#define PIN_GP12            12                  // Debug / RFU
#define PIN_GP13            13                  // Debug / RFU
#define PIN_TIME_MEASURE    12                  // Core1 PDM処理時間計測用
#define PIN_PIOT_MEASURE    13                  // Core1 PIO処理時間計測用
#define PIN_GP18            18                  // Debug / RFU
#define PIN_GP26            26                  // Debug / RFU
#define PIN_GP27            27                  // Debug / RFU
#define PIN_OUTPUT_RP       14                  // RCh P
#define PIN_OUTPUT_RN       15                  // RCh N
#define PIN_OUTPUT_LP       16                  // LCh P
#define PIN_OUTPUT_LN       17                  // LCh N

#define PIN_I2S_SDO         19                  // to Raspberry pi
#define PIN_I2S_DATA_OUT    19                  // to Raspberry pi
#define PIN_I2S_SDI         20                  // from Raspberry pi
#define PIN_I2S_DATA_IN     20                  // from Raspberry pi
#define PIN_I2S_LRCK_IO     21                  // from/to Raspberry pi
#define PIN_I2S_LRCK_IN     21                  // from/to Raspberry pi
#define PIN_I2S_LRCK_OUT    21                  // from/to Raspberry pi
#define PIN_I2S_LRCK        21                  // from/to Raspberry pi
#define PIN_I2S_BCK_IO      22                  // from/to Raspberry pi
#define PIN_I2S_BCK_IN      22                  // from/to Raspberry pi
#define PIN_I2S_BCK_OUT     22                  // from/to Raspberry pi
#define PIN_I2S_BCK         22                  // from/to Raspberry pi

#define PIN_DCDC_PWM        23                  // DCDC PFM_PWM Control pin
#define PIN_VBUS_DETECT     24                  // VBUS Detect pin ; GPIO24 = VBUS * 10k / (5k6 + 10k)
#define PIN_PICO_LED        25                  // PICO on-board LED Pin

//#define PIN_FS48			25                  // fs=48k flag (as the on-board LED Pin)
#define PIN_FS48			28                  // fs=48k flag (as PICO_DAC_on-board LED Pin)
#define PIN_DAC_STATE_GPIO	18
#define PIN_DAC_LED			28                  // fs=48k flag (as PICO_DAC_on-board LED Pin)
#define PIN_VSYS_LEVEL		29                  // VSYS Level Monitor pin;

// ピン名定義(回路図インスタンスと同一)
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
