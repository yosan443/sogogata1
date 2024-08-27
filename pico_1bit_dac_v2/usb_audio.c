/*
 * usb_sound_card
 *
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file usb_audio.c
 * @brief pico_1bit_dac_v2用 (pico_1bit_dac_HR2より派生)
 * @author geachlab, Yasushi MARUISHI
 * @version 0.01
 * @date 2023.02.21
 * @note USB DAC機能 (usb_sound_card 由来)
 * 旧 pico_1bit_dac.c の USB Audio Class処理部分を分離
 */

// USB Audio Packet受信デバッグ時に1とする
#if 0
#define DEBUG_PIN(x, y) gpio_put((x), (y))
#else
#define DEBUG_PIN(x, y)	/*処理なし*/
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/usb_device.h"
#include "pico/multicore.h"
#include "lufa/AudioClassCommon.h"
#include "hardware/sync.h"

#include "audio_state.h"
#include "bsp.h"
#include "dsp.h"
#include "simple_queue.h"
#include "pdm_output.h"

// todo make descriptor strings should probably belong to the configs
static char *descriptor_strings[] =
        {
                "Raspberry Pi",
                "Pico Examples Sound Card",
                "0123456789AB"
        };

// todo fix these
#define VENDOR_ID   0x2e8au
#define PRODUCT_ID  0xfeddu
//#define PRODUCT_ID  (0xfeddu + 1)

#define AUDIO_OUT_ENDPOINT  0x01U
#define AUDIO_IN_ENDPOINT   0x82U

#undef AUDIO_SAMPLE_FREQ
#define AUDIO_SAMPLE_FREQ(frq) (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))

#define AUDIO_MAX_SAMPLE_NUM	(192 + 1)	// (192kHz/1kHz +1)
#define AUDIO_MAX_PACKET_SIZE	(2 * 2 * AUDIO_MAX_SAMPLE_NUM) // 2ch * 2byte/ch * (192kHz/1kHz +1)

#define FEATURE_MUTE_CONTROL 1u
#define FEATURE_VOLUME_CONTROL 2u

#define ENDPOINT_FREQ_CONTROL 1u

struct audio_device_config {
	struct usb_configuration_descriptor descriptor;
	struct usb_interface_descriptor ac_interface;
	struct __packed {
		USB_Audio_StdDescriptor_Interface_AC_t core;
		USB_Audio_StdDescriptor_InputTerminal_t input_terminal;
		USB_Audio_StdDescriptor_FeatureUnit_t feature_unit;
		USB_Audio_StdDescriptor_OutputTerminal_t output_terminal;
	} ac_audio;
	struct usb_interface_descriptor as_zero_interface;
	struct usb_interface_descriptor as_op_interface;
	struct __packed {
		USB_Audio_StdDescriptor_Interface_AS_t streaming;
		struct __packed {
			USB_Audio_StdDescriptor_Format_t core;
			USB_Audio_SampleFreq_t freqs[6];	// 44.1/48/88.2/96/176.4/192kHz対応のため配列数を2->6に増やす
		} format;
	} as_audio;
	struct __packed {
		struct usb_endpoint_descriptor_long core;
		USB_Audio_StdDescriptor_StreamEndpoint_Spc_t audio;
	} ep1;
	struct usb_endpoint_descriptor_long ep2;
	// Alternate2 = 24bit再生用の定義を追記する
	struct usb_interface_descriptor as_op_interface_2;
	struct __packed {
		USB_Audio_StdDescriptor_Interface_AS_t streaming;
		struct __packed {
			USB_Audio_StdDescriptor_Format_t core;
			USB_Audio_SampleFreq_t freqs[4];	// <- 44.1/48/88.2/96kHz対応のため配列数を4とする
		} format;
	} as_audio_2;
	struct __packed {
		struct usb_endpoint_descriptor_long core;
		USB_Audio_StdDescriptor_StreamEndpoint_Spc_t audio;
	} ep1_2;
	struct usb_endpoint_descriptor_long ep2_2;
};

static const struct audio_device_config audio_device_config = {
	.descriptor = {
		.bLength             = sizeof(audio_device_config.descriptor),
		.bDescriptorType     = DTYPE_Configuration,
		.wTotalLength        = sizeof(audio_device_config),
		.bNumInterfaces      = 2,
		.bConfigurationValue = 0x01,
		.iConfiguration      = 0x00,
		.bmAttributes        = 0x80,
//		.bMaxPower           = 0xfa,	//0x32(100mA)->0xfa(500mA) アナログ回路負荷用に電流を増やしておく
		.bMaxPower           = 0x32,	//0x32(100mA)->0xfa(500mA)->0x32(100mA) Apple A1619(Lightning - USB 3カメラアダプタ)対応
	},
	.ac_interface = {
		.bLength            = sizeof(audio_device_config.ac_interface),
		.bDescriptorType    = DTYPE_Interface,
		.bInterfaceNumber   = 0x00,
		.bAlternateSetting  = 0x00,
		.bNumEndpoints      = 0x00,
		.bInterfaceClass    = AUDIO_CSCP_AudioClass,
		.bInterfaceSubClass = AUDIO_CSCP_ControlSubclass,
		.bInterfaceProtocol = AUDIO_CSCP_ControlProtocol,
		.iInterface         = 0x00,
	},
	.ac_audio = {
		.core = {
			.bLength = sizeof(audio_device_config.ac_audio.core),
			.bDescriptorType = AUDIO_DTYPE_CSInterface,
			.bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_Header,
			.bcdADC = VERSION_BCD(1, 0, 0),
			.wTotalLength = sizeof(audio_device_config.ac_audio),
			.bInCollection = 1,
			.bInterfaceNumbers = 1,
		},
		.input_terminal = {
			.bLength = sizeof(audio_device_config.ac_audio.input_terminal),
			.bDescriptorType = AUDIO_DTYPE_CSInterface,
			.bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_InputTerminal,
			.bTerminalID = 1,
			.wTerminalType = AUDIO_TERMINAL_STREAMING,
			.bAssocTerminal = 0,
			.bNrChannels = 2,
			.wChannelConfig = AUDIO_CHANNEL_LEFT_FRONT | AUDIO_CHANNEL_RIGHT_FRONT,
			.iChannelNames = 0,
			.iTerminal = 0,
		},
		.feature_unit = {
			.bLength = sizeof(audio_device_config.ac_audio.feature_unit),
			.bDescriptorType = AUDIO_DTYPE_CSInterface,
			.bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_Feature,
			.bUnitID = 2,
			.bSourceID = 1,
			.bControlSize = 1,
			.bmaControls = {AUDIO_FEATURE_MUTE | AUDIO_FEATURE_VOLUME, 0, 0},
			.iFeature = 0,
		},
		.output_terminal = {
			.bLength = sizeof(audio_device_config.ac_audio.output_terminal),
			.bDescriptorType = AUDIO_DTYPE_CSInterface,
			.bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_OutputTerminal,
			.bTerminalID = 3,
			.wTerminalType = AUDIO_TERMINAL_OUT_SPEAKER,
			.bAssocTerminal = 0,
			.bSourceID = 2,
			.iTerminal = 0,
		},
	},
	.as_zero_interface = {
		.bLength            = sizeof(audio_device_config.as_zero_interface),
		.bDescriptorType    = DTYPE_Interface,
		.bInterfaceNumber   = 0x01,
		.bAlternateSetting  = 0x00,
		.bNumEndpoints      = 0x00,
		.bInterfaceClass    = AUDIO_CSCP_AudioClass,
		.bInterfaceSubClass = AUDIO_CSCP_AudioStreamingSubclass,
		.bInterfaceProtocol = AUDIO_CSCP_ControlProtocol,
		.iInterface         = 0x00,
	},
	.as_op_interface = {
		.bLength            = sizeof(audio_device_config.as_op_interface),
		.bDescriptorType    = DTYPE_Interface,
		.bInterfaceNumber   = 0x01,
		.bAlternateSetting  = 0x01,
		.bNumEndpoints      = 0x02,
		.bInterfaceClass    = AUDIO_CSCP_AudioClass,
		.bInterfaceSubClass = AUDIO_CSCP_AudioStreamingSubclass,
		.bInterfaceProtocol = AUDIO_CSCP_ControlProtocol,
		.iInterface         = 0x00,
	},
	.as_audio = {
		.streaming = {
			.bLength = sizeof(audio_device_config.as_audio.streaming),
			.bDescriptorType = AUDIO_DTYPE_CSInterface,
			.bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_General,
			.bTerminalLink = 1,
			.bDelay = 1,
			.wFormatTag = 1, // PCM
		},
		.format = {
			.core = {
				.bLength = sizeof(audio_device_config.as_audio.format),
				.bDescriptorType = AUDIO_DTYPE_CSInterface,
				.bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_FormatType,
				.bFormatType = 1,
				.bNrChannels = 2,
				.bSubFrameSize = 2,
				.bBitResolution = 16,
				.bSampleFrequencyType = count_of(audio_device_config.as_audio.format.freqs),
			},
			.freqs = {
				AUDIO_SAMPLE_FREQ(44100),
				AUDIO_SAMPLE_FREQ(48000),
				AUDIO_SAMPLE_FREQ(88200),	// ハイレゾ対応
				AUDIO_SAMPLE_FREQ(96000),	// ハイレゾ対応
				AUDIO_SAMPLE_FREQ(176400),	// ハイレゾ対応
				AUDIO_SAMPLE_FREQ(192000),	// ハイレゾ対応
			},
		},
	},
	.ep1 = {
		.core = {
			.bLength          = sizeof(audio_device_config.ep1.core),
			.bDescriptorType  = DTYPE_Endpoint,
			.bEndpointAddress = AUDIO_OUT_ENDPOINT,
			.bmAttributes     = 5,
			.wMaxPacketSize   = AUDIO_MAX_PACKET_SIZE,
			.bInterval        = 1,
			.bRefresh         = 0,
			.bSyncAddr        = AUDIO_IN_ENDPOINT,
		},
		.audio = {
			.bLength = sizeof(audio_device_config.ep1.audio),
			.bDescriptorType = AUDIO_DTYPE_CSEndpoint,
			.bDescriptorSubtype = AUDIO_DSUBTYPE_CSEndpoint_General,
			.bmAttributes = 1,
			.bLockDelayUnits = 0,
			.wLockDelay = 0,
		}
	},
	.ep2 = {
		.bLength          = sizeof(audio_device_config.ep2),
		.bDescriptorType  = 0x05,
		.bEndpointAddress = AUDIO_IN_ENDPOINT,
		.bmAttributes     = 0x01,
		.wMaxPacketSize   = 3,
		.bInterval        = 0x01,
		.bRefresh         = 3,	//8ms
		.bSyncAddr        = 0,
	},

	.as_op_interface_2 = {
		.bLength            = sizeof(audio_device_config.as_op_interface_2),
		.bDescriptorType    = DTYPE_Interface,
		.bInterfaceNumber   = 0x01,
		.bAlternateSetting  = 0x02,
		.bNumEndpoints      = 0x02,
		.bInterfaceClass    = AUDIO_CSCP_AudioClass,
		.bInterfaceSubClass = AUDIO_CSCP_AudioStreamingSubclass,
		.bInterfaceProtocol = AUDIO_CSCP_ControlProtocol,
		.iInterface         = 0x00,
	},
	.as_audio_2 = {
		.streaming = {
			.bLength = sizeof(audio_device_config.as_audio_2.streaming),
			.bDescriptorType = AUDIO_DTYPE_CSInterface,
			.bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_General,
			.bTerminalLink = 1,
			.bDelay = 1,
			.wFormatTag = 1, // PCM
		},
		.format = {
			.core = {
				.bLength = sizeof(audio_device_config.as_audio_2.format),
				.bDescriptorType = AUDIO_DTYPE_CSInterface,
				.bDescriptorSubtype = AUDIO_DSUBTYPE_CSInterface_FormatType,
				.bFormatType = 1,
				.bNrChannels = 2,
				.bSubFrameSize = 3,		// 24bit = 3byte
				.bBitResolution = 24,	// 24bit
				.bSampleFrequencyType = count_of(audio_device_config.as_audio_2.format.freqs),
			},
			.freqs = {
				AUDIO_SAMPLE_FREQ(44100),	// ハイレゾ対応 
				AUDIO_SAMPLE_FREQ(48000),	// ハイレゾ対応
				AUDIO_SAMPLE_FREQ(88200),	// ハイレゾ対応
				AUDIO_SAMPLE_FREQ(96000)	// ハイレゾ対応
			},
		},
	},
	.ep1_2 = {
		.core = {
			.bLength          = sizeof(audio_device_config.ep1_2.core),
			.bDescriptorType  = DTYPE_Endpoint,
			.bEndpointAddress = AUDIO_OUT_ENDPOINT,
			.bmAttributes     = 5,
			.wMaxPacketSize   = AUDIO_MAX_PACKET_SIZE,
			.bInterval        = 1,
			.bRefresh         = 0,
			.bSyncAddr        = AUDIO_IN_ENDPOINT,
		},
		.audio = {
			.bLength = sizeof(audio_device_config.ep1_2.audio),
			.bDescriptorType = AUDIO_DTYPE_CSEndpoint,
			.bDescriptorSubtype = AUDIO_DSUBTYPE_CSEndpoint_General,
			.bmAttributes = 1,
			.bLockDelayUnits = 0,
			.wLockDelay = 0,
		}
	},
	.ep2_2 = {
		.bLength          = sizeof(audio_device_config.ep2_2),
		.bDescriptorType  = 0x05,
		.bEndpointAddress = AUDIO_IN_ENDPOINT,
		.bmAttributes     = 0x01,
		.wMaxPacketSize   = 3,
		.bInterval        = 0x01,
		.bRefresh         = 3,	//8ms
		.bSyncAddr        = 0,
	},
};

static struct usb_interface ac_interface;
static struct usb_interface as_op_interface;
static struct usb_endpoint ep_op_out, ep_op_sync;

static const struct usb_device_descriptor boot_device_descriptor = {
	.bLength            = 18,
	.bDescriptorType    = 0x01,
	.bcdUSB             = 0x0110,
	.bDeviceClass       = 0x00,
	.bDeviceSubClass    = 0x00,
	.bDeviceProtocol    = 0x00,
	.bMaxPacketSize0    = 0x40,
	.idVendor           = VENDOR_ID,
	.idProduct          = PRODUCT_ID,
	.bcdDevice          = 0x0200,
	.iManufacturer      = 0x01,
	.iProduct           = 0x02,
	.iSerialNumber      = 0x03,
	.bNumConfigurations = 0x01,
};

const char *_get_descriptor_string(uint index) {
    if (index <= count_of(descriptor_strings)) {
        return descriptor_strings[index - 1];
    } else {
        return "";
    }
}

extern audio_state_t audio_state;

// バッファ長制限 ・Win10 バッファオーバーラン防止処理
// Win10でfs切替直後の再生時、Feedback規定数を超える
// サンプル数を受信する事があり、次の二重対策をした。
// 1)最大バッファ長を増やす(simple_queue.h)
// 2)サンプル数を最大バッファ長に制限する(以下) 
static inline void buf_len_limiter(uint fs, uint* len){
	uint len_limit;
	switch(fs){
		case 192000:
		case 176400:
			len_limit = QUEUE_WIDTH /2 /2;
			break;
		case  96000:
		case  88200:
			len_limit = QUEUE_WIDTH /2 /4;
			break;
		case  48000:
		case  44100:
		default:
			len_limit = QUEUE_WIDTH /2 /8;
			break;	// QUEUE_WIDTH は2Chステレオ x8 OverSampling幅を持っているため /2/8としている
	}
	if (*len > len_limit){
		*len = len_limit;
	} 
}


// USB ep(エンドポイント)→ローカルバッファコピー処理
// ep上の24/16bit型敷き詰めデータを32bit型データへ整形
// epアクセスを16bit->32bitに変更し、処理高速化・低消費電力化
static inline void usb_ep_to_buf_copy(uint bit_depth, uint32_t* ep, int32_t* buf, uint* len){
	uint sample_num;
	switch(bit_depth){
	  case 24 :
		*len /= 3;	// (3byte(24bit) /ch)
		buf_len_limiter(audio_state.fs, len);	//処理周波数に応じたデータ数に制限
		sample_num = *len / 2 + 1;	
		while(sample_num--) {	// 24bit敷き詰めデータ→32bit変換
			uint w0 = *ep++;	// Ch1_A[ 7: 0],Ch0_A[23: 0]
			uint w1 = *ep++;	// Ch0_B[15: 0],Ch1_A[23: 8]
			uint w2 = *ep++;	// Ch1_B[23: 0],Ch0_B[23:16]
			*buf++ = (int)(          (w0 << 8)) >> 8;	// Ch0_A
			*buf++ = (int)((w0 >>16)|(w1 <<16)) >> 8;	// Ch1_A
			*buf++ = (int)((w1 >> 8)|(w2 <<24)) >> 8;	// Ch0_B
			*buf++ = (int)(          (w2     )) >> 8;	// Ch1_B
		}
		break;
	  case 16 :
	  default :
		*len /= 2;	// (2byte(16bit) /ch)
		buf_len_limiter(audio_state.fs, len);	//処理周波数に応じたデータ数に制限
		sample_num = *len;
		while(sample_num--) {
			uint32_t t = *ep++;	// Ch1[15:00],Ch0[15:00]
			*buf++ = (int)(t << 16       ) >> 8;	// Ch0 int32_t(int24_t相当)化
			*buf++ = (int)(t & 0xffff0000) >> 8;	// Ch1 int32_t(int24_t相当)化
		//	*buf++ = sample_num << 16;		// RCh 周波数FeedBack追従試験用。LChに0dBFS 1kHz Sin波を入力し、LRのリサージュ波形を観測する。
		}
		break;
	}
}

/* ※注 usb_buffer->data_len の型宣言について
  参照元の usb_buffer 構造体はデータ長を uint8_t で宣言しているが、USB1.x 規格では
  wMaxPacketSize が0~1023の値を取りうるため、uint16_t または uint32_tとすべき。以下修正例 :
/pico/pico-extras/src/rp2_common/usb_device/include/pico/usb_device.h
struct usb_buffer {
    uint8_t *data;
    uint16_t data_len;	//  uint8_t data_len;
    uint16_t data_max;	//  uint8_t data_max;
   // then...
   bool valid; // aka user owned
};*/

// UAC Audio Packet受信時のデータ処理
// 受信済みデータをUSBエンドポイントからdspバッファにコピー、
// 音量処理、オーバーサンプリングを実施し、Core0~1間キューに積む。
static void _as_audio_packet(struct usb_endpoint *ep) {
	// 前回データを処理中だった場合は処理中止
	if(audio_state.data_received) {
		usb_grow_transfer(ep->current_transfer, 1);
		usb_packet_done(ep);
		return;
	}
	struct usb_buffer *usb_buffer = usb_current_out_packet_buffer(ep);

	// ui8ポインタをui32にキャスト 
	uint32_t *in = (uint32_t *)usb_buffer->data;
	// ※ui8データ数をi16データ数(1/2)に変換
	// 後段でbit_depth(/2、/3)に応じて再計算する 
	uint len = (usb_buffer->data_len) / 2;

	// 入力fsに応じたdspバッファポインタを取得
	int32_t *dsp_buf = get_dsp_buf_pointer(audio_state.fs);
	// usb epデータコピー　
	usb_ep_to_buf_copy(audio_state.bit_depth, in, dsp_buf, &len);

	// usb epデータコピー完了処理
	// これを先に完了しておくことで、1SOF期間の大半を演算に使用可能
	usb_grow_transfer(ep->current_transfer, 1);
	usb_packet_done(ep);

	audio_state.len = len;
	audio_state.dsp_buf = dsp_buf;
	audio_state.data_received = true;
}

// UAC Sync Packet送信時のfS(サンプリング周波数)フィードバック処理
// ハイレゾ化に伴い、フィードバック幅をfsに応じて可変するよう変更。
// 旧処理では フィードバック周波数演算をint32で行っていたが、高精度化のためfloatに変更。
// 可変幅FS_FEEDBACK_RATIO_F は fsに対する比(float)で定義。
// FS_FEEDBACK_RATIO_F = 0.001, fs = 48000の場合、 48000 * 0.001 = ±48.0Hzとなる
/*  適応Feedback
	キューサンプル数がターゲット水位に接近すると、フィードバック量が小さくなるよう制御する。
	これによりキューサンプル数がターゲット水位で安定し、目標周波数に平衡する
							adjust_value
	|+adjust_value_max _____  A
	|                       \ :
	|                        \:
	0 +-------------------------+------------------> queue_samples
	|                         :\              
	|                         : \_____-adjust_value_max
	|                         :
	0              QUEUE_TARGET_WATERLEVEL
*/

#define FS_FEEDBACK_RATIO_F		((float)0.001)

static void _as_sync_packet(struct usb_endpoint *ep) {
    assert(ep->current_transfer);
    struct usb_buffer *buffer = usb_current_in_packet_buffer(ep);
    assert(buffer->data_max >= 3);
    buffer->data_len = 3;

	// Feedbackパラメタ計算
	// 精度確保のためfloatを利用。float演算は遅いため、fs(sampling frequency)更新時のみパラメタ系を再計算する。
	static float adjust_value_max;
	static float true_playback_fs;
	static float queue_samples_waterlevel;
	static uint fs = 0;	// fs更新判定用 
	DEBUG_PIN(GP12,1);
	if (fs != audio_state.fs){
		fs = audio_state.fs;
		adjust_value_max = (float)fs * FS_FEEDBACK_RATIO_F;		// Feedback	制限(上限・下限)周波数
		queue_samples_waterlevel = (float)(fs * QUEUE_TARGET_WATERLEVEL) * 0.001; // 1SOF内の水位定義のため、1ms = 0.001sを乗じる 
		true_playback_fs = get_true_playback_fs(fs);				// 真のDACシステム再生周波数
	}
	// Feedback調整値計算と調整値制限
	float adjust_value = queue_samples_waterlevel - (float)get_input_fs_referred_queue_samples(fs);
	if		(adjust_value > +adjust_value_max) adjust_value = +adjust_value_max;	// Feedback上限
	else if (adjust_value < -adjust_value_max) adjust_value = -adjust_value_max;	// Feedback下限
 
	float feedback_fs = true_playback_fs + adjust_value;
	uint feedback = (uint)(feedback_fs * (16384.0 / 1000.0));	// 旧処理 : feedback = ((audio_state.fs + adjust_value) << 14u) / 1000u;

    buffer->data[0] = feedback;
    buffer->data[1] = feedback >> 8u;
    buffer->data[2] = feedback >> 16u;

    usb_grow_transfer(ep->current_transfer, 1);
    usb_packet_done(ep);
	DEBUG_PIN(GP12,0);

}

static const struct usb_transfer_type as_transfer_type = {
        .on_packet = _as_audio_packet,
        .initial_packet_count = 1,
};

static const struct usb_transfer_type as_sync_transfer_type = {
        .on_packet = _as_sync_packet,
        .initial_packet_count = 1,
};

static struct usb_transfer as_transfer;
static struct usb_transfer as_sync_transfer;

static bool do_get_current(struct usb_setup_packet *setup) {
//    usb_debug("AUDIO_REQ_GET_CUR\n");

    if ((setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK) == USB_REQ_TYPE_RECIPIENT_INTERFACE) {
        switch (setup->wValue >> 8u) {
            case FEATURE_MUTE_CONTROL: {
                usb_start_tiny_control_in_transfer(audio_state.mute, 1);
                return true;
            }
            case FEATURE_VOLUME_CONTROL: {
                /* Current volume. See UAC Spec 1.0 p.77 */
                usb_start_tiny_control_in_transfer(audio_state.volume, 2);
                return true;
            }
        }
    } else if ((setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK) == USB_REQ_TYPE_RECIPIENT_ENDPOINT) {
        if ((setup->wValue >> 8u) == ENDPOINT_FREQ_CONTROL) {
            /* Current frequency */
            usb_start_tiny_control_in_transfer(audio_state.fs, 3);
            return true;
        }
    }
    return false;
}


#define CENTER_VOLUME_INDEX	90			// 元の値(91dB)に最も近い6dBの倍数値に変更
#define VOLUME_SHIFT_STEP	96			// 6dB / 96ステップ毎にVolumeを1bitずつシフトする

// 0~6dB(-6~0dB)区間音量係数テーブル
/**	-6dB to 0dB / 96 step volume table
*  input resolution : 6dB / 96step = 0.0625 [dB/step]
*  input range      : 0 to 95 (mapped -6 to -0.0625dB)
*  output range     : 128 to 255 [LSB/0.0625dB]
*  curve formula    : y(x) = int(128 * 2^(x / 96) + 0.5)
*/
const uint8_t db_to_vol_6db_span[VOLUME_SHIFT_STEP] = {
	128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, // -6 to -5.0625dB
	144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 156, 157, 158, 159, 160, // -5 to -4.0625dB
	161, 162, 164, 165, 166, 167, 168, 170, 171, 172, 173, 175, 176, 177, 178, 180, // -4 to -3.0625dB
	181, 182, 184, 185, 186, 188, 189, 190, 192, 193, 195, 196, 197, 199, 200, 202, // -3 to -2.0625dB
	203, 205, 206, 208, 209, 211, 212, 214, 215, 217, 218, 220, 222, 223, 225, 226, // -2 to -1.0625dB
	228, 230, 231, 233, 235, 236, 238, 240, 242, 243, 245, 247, 249, 251, 252, 254, // -1 to -0.0625dB
};

#define ENCODE_DB(x) ((int16_t)((x)*256))

#define MIN_VOLUME           ENCODE_DB(-CENTER_VOLUME_INDEX)
#define DEFAULT_VOLUME       ENCODE_DB(0)
#define MAX_VOLUME           ENCODE_DB(0)
#define VOLUME_RESOLUTION    ENCODE_DB(6.0 / 96.0)

static void audio_set_volume(int16_t volume) {
    audio_state.volume = volume;
	if (volume > 0) volume = 0;					// Upper Clip to 0[dB]
    volume += CENTER_VOLUME_INDEX * 256;		// Shift volume level -90~0 -> 0 ~ 90
    if (volume < 0) volume = 0;					// Lower Clip to 0[dB] 
	volume >>= 4;								// convert 1dB/256step to 1dB/16step
	audio_state.vol_shift =	(uint32_t)(7 + CENTER_VOLUME_INDEX / 6 - volume / VOLUME_SHIFT_STEP);
	audio_state.vol_mul   =	 (int32_t)db_to_vol_6db_span[volume % VOLUME_SHIFT_STEP];
}

static bool do_get_minimum(struct usb_setup_packet *setup) {
//    usb_debug("AUDIO_REQ_GET_MIN\n");
    if ((setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK) == USB_REQ_TYPE_RECIPIENT_INTERFACE) {
        switch (setup->wValue >> 8u) {
            case FEATURE_VOLUME_CONTROL: {
                usb_start_tiny_control_in_transfer(MIN_VOLUME, 2);
    printf("setup : Min_volume = %04x, %f\n", MIN_VOLUME, (float)((int32_t)(65536-MIN_VOLUME)) / 256);
                return true;
            }
        }
    }
    return false;
}

static bool do_get_maximum(struct usb_setup_packet *setup) {
//    usb_debug("AUDIO_REQ_GET_MAX\n");
    if ((setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK) == USB_REQ_TYPE_RECIPIENT_INTERFACE) {
        switch (setup->wValue >> 8u) {
            case FEATURE_VOLUME_CONTROL: {
                usb_start_tiny_control_in_transfer(MAX_VOLUME, 2);
    printf("setup : Max_volume = %04x, %f\n", MAX_VOLUME, (float)((int32_t)MAX_VOLUME) / 256);
                return true;
            }
        }
    }
    return false;
}

static bool do_get_resolution(struct usb_setup_packet *setup) {
//    usb_debug("AUDIO_REQ_GET_RES\n");
    if ((setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK) == USB_REQ_TYPE_RECIPIENT_INTERFACE) {
        switch (setup->wValue >> 8u) {
            case FEATURE_VOLUME_CONTROL: {
                usb_start_tiny_control_in_transfer(VOLUME_RESOLUTION, 2);
    printf("setup : Resolution = %04x, %f\n", VOLUME_RESOLUTION, (float)((int32_t)VOLUME_RESOLUTION) / 256);
                return true;
            }
        }
    }
    return false;
}

static struct audio_control_cmd {
    uint8_t cmd;
    uint8_t type;
    uint8_t cs;
    uint8_t cn;
    uint8_t unit;
    uint8_t len;
} audio_control_cmd_t;

static void _audio_reconfigure() {
    switch (audio_state.fs) {
        case 44100:
        case 48000:
        case 88200:		// ハイレゾ対応
        case 96000:		// ハイレゾ対応
        case 176400:	// ハイレゾ対応
        case 192000:	// ハイレゾ対応
            break;
        default:
            audio_state.fs = 0;				// 定義済みのFormatでなければ無効値とする
    }
}


static void audio_cmd_packet(struct usb_endpoint *ep) {
    assert(audio_control_cmd_t.cmd == AUDIO_REQ_SetCurrent);
    struct usb_buffer *buffer = usb_current_out_packet_buffer(ep);
    audio_control_cmd_t.cmd = 0;
    if (buffer->data_len >= audio_control_cmd_t.len) {
        if (audio_control_cmd_t.type == USB_REQ_TYPE_RECIPIENT_INTERFACE) {
            switch (audio_control_cmd_t.cs) {
                case FEATURE_MUTE_CONTROL: {
                    audio_state.mute = buffer->data[0];
                    usb_warn("Set Mute %d\n", buffer->data[0]);
                    break;
                }
                case FEATURE_VOLUME_CONTROL: {
                    audio_set_volume(*(int16_t *) buffer->data);
                    break;
                }
            }

        } else if (audio_control_cmd_t.type == USB_REQ_TYPE_RECIPIENT_ENDPOINT) {
            if (audio_control_cmd_t.cs == ENDPOINT_FREQ_CONTROL) {
                uint32_t new_freq = (*(uint32_t *) buffer->data) & 0x00ffffffu;
                usb_warn("Set freq %d\n", new_freq == 0xffffffu ? -1 : (int) new_freq);

                if (audio_state.fs != new_freq) {
                    audio_state.fs = new_freq;
                    _audio_reconfigure();
					hbf_oversampler_reset();
					audio_state.group_48k_src = get_group_48k(new_freq);
					audio_state.group_48k_dac = audio_state.group_48k_src;
//					set_dac_fs_group_48k(audio_state.group_48k_src);
					audio_state.format_updated = true;
                }
            }
        }
    }
    usb_start_empty_control_in_transfer_null_completion();
    // todo is there error handling?
}


static const struct usb_transfer_type _audio_cmd_transfer_type = {
        .on_packet = audio_cmd_packet,
        .initial_packet_count = 1,
};

static bool as_set_alternate(struct usb_interface *interface, uint alt) {
    assert(interface == &as_op_interface);
    switch(alt){
        case 2 : audio_state.bit_depth = 24; break;
        case 1 : audio_state.bit_depth = 16; break;
        case 0 :
        default: break; // Alternate 0 の場合、何もせず直前のbit_depthを保持
	}
    usb_warn("SET ALTERNATE %d, bit_depth = %d\n", alt, audio_state.bit_depth);
//    change_dac_state_gpio();
	hbf_oversampler_reset();
    return alt < 3;	// Alternate 0~2を正常値とする
}

static bool do_set_current(struct usb_setup_packet *setup) {
#if 0
#ifndef NDEBUG
    usb_warn("AUDIO_REQ_SET_CUR\n");
#endif
#endif
    if (setup->wLength && setup->wLength < 64) {
        audio_control_cmd_t.cmd = AUDIO_REQ_SetCurrent;
        audio_control_cmd_t.type = setup->bmRequestType & USB_REQ_TYPE_RECIPIENT_MASK;
        audio_control_cmd_t.len = (uint8_t) setup->wLength;
        audio_control_cmd_t.unit = setup->wIndex >> 8u;
        audio_control_cmd_t.cs = setup->wValue >> 8u;
        audio_control_cmd_t.cn = (uint8_t) setup->wValue;
        usb_start_control_out_transfer(&_audio_cmd_transfer_type);
        return true;
    }
    return false;
}

static bool ac_setup_request_handler(__unused struct usb_interface *interface, struct usb_setup_packet *setup) {
    setup = __builtin_assume_aligned(setup, 4);
    if (USB_REQ_TYPE_TYPE_CLASS == (setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK)) {
        switch (setup->bRequest) {
            case AUDIO_REQ_SetCurrent:
                return do_set_current(setup);

            case AUDIO_REQ_GetCurrent:
                return do_get_current(setup);

            case AUDIO_REQ_GetMinimum:
                return do_get_minimum(setup);

            case AUDIO_REQ_GetMaximum:
                return do_get_maximum(setup);

            case AUDIO_REQ_GetResolution:
                return do_get_resolution(setup);

            default:
                break;
        }
    }
    return false;
}

bool _as_setup_request_handler(__unused struct usb_endpoint *ep, struct usb_setup_packet *setup) {
    setup = __builtin_assume_aligned(setup, 4);
    if (USB_REQ_TYPE_TYPE_CLASS == (setup->bmRequestType & USB_REQ_TYPE_TYPE_MASK)) {
        switch (setup->bRequest) {
            case AUDIO_REQ_SetCurrent:
                return do_set_current(setup);

            case AUDIO_REQ_GetCurrent:
                return do_get_current(setup);

            case AUDIO_REQ_GetMinimum:
                return do_get_minimum(setup);

            case AUDIO_REQ_GetMaximum:
                return do_get_maximum(setup);

            case AUDIO_REQ_GetResolution:
                return do_get_resolution(setup);

            default:
                break;
        }
    }
    return false;
}

// USB初期化と出音ループ
// 旧名は usb_sound_card_init()
void usb_init() {
    //msd_interface.setup_request_handler = msd_setup_request_handler;
    usb_interface_init(&ac_interface, &audio_device_config.ac_interface, NULL, 0, true);
    ac_interface.setup_request_handler = ac_setup_request_handler;

    static struct usb_endpoint *const op_endpoints[] = {
            &ep_op_out, &ep_op_sync
    };
    usb_interface_init(&as_op_interface, &audio_device_config.as_op_interface, op_endpoints, count_of(op_endpoints),
                       true);
    as_op_interface.set_alternate_handler = as_set_alternate;
    ep_op_out.setup_request_handler = _as_setup_request_handler;
    as_transfer.type = &as_transfer_type;
    usb_set_default_transfer(&ep_op_out, &as_transfer);
    as_sync_transfer.type = &as_sync_transfer_type;
    usb_set_default_transfer(&ep_op_sync, &as_sync_transfer);

    static struct usb_interface *const boot_device_interfaces[] = {
            &ac_interface,
            &as_op_interface,
    };
    __unused struct usb_device *device = usb_device_init(&boot_device_descriptor, &audio_device_config.descriptor,
                                                         boot_device_interfaces, count_of(boot_device_interfaces),
                                                         _get_descriptor_string);
    assert(device);
    audio_set_volume(DEFAULT_VOLUME);
    _audio_reconfigure();
//    device->on_configure = _on_configure;
    usb_device_start();
	// 起動時右Chポップノイズ対策
	// RP2040初期品(RP2040B0, RP2040B1)では、USB初期化時に Errata RP2040―E5 Workaround により
	// GPIO15 がトグルし、アナログ右出力にポップノイズが発生する。
	// USB初期化後に250ms waitを入れ、トグル期間はGPIO15をHi-z(初期値)のままとすることで、これを回避する。
	busy_wait_ms(250);

//	printf("HAHA %04x %04x %04x %04x\n", MIN_VOLUME, DEFAULT_VOLUME, MAX_VOLUME, VOLUME_RESOLUTION);
	puts("USB init done.");
}
