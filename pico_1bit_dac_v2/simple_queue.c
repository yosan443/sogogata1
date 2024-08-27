/**
 * @file simple_queue.c
 * @brief Core0~Core1間 PCMキュー fs = 384kHz対応版
 * @version 0.2
 * @date 2022-10-17
 * @copyright Copyright (c) 2022
 * @note
 * 旧処理→新処理で、simple_queue内に存在したDSP処理(USB ep引取～音量処理～オーバーサンプル前置フィルタ)を呼び出し側のpico_1bit_dacに移動。
 */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/interp.h"
#include "hardware/sync.h"

#include "simple_queue.h"

#define SPINLOCK_ID_AUDIO_QUEUE (16 + 0)

static spin_lock_t* queue_spin_lock;
static int32_t audio_queue[QUEUE_DEPTH][QUEUE_WIDTH];	// 1段≒1ms分のサンプルデータ
static uint32_t audio_queue_len[QUEUE_DEPTH];			// QUEUE_DEPTH段の各queueが蓄積しているサンプル数

/**
 * バッファ蓄積管理の変数
 */
static uint32_t dequeue_pos;
static uint32_t enqueue_pos;
static uint32_t queue_length;	// キューされた段数
static uint32_t queue_samples;	// キューされたサンプル数

/**
 * キューのリセットと初期化
 */
void queue_reset(void)
{
	dequeue_pos = 0;
	enqueue_pos = 0;
	queue_length = 0;
	queue_samples = 0;
}

void queue_init(void)
{
	queue_spin_lock = spin_lock_init(SPINLOCK_ID_AUDIO_QUEUE);
	queue_reset();
}

/**
 * キューからサンプルを引き取る
 */
inline bool dequeue(int32_t** buf, uint32_t* len)
{
    if (get_queue_length()) 
    {
        *len = audio_queue_len[dequeue_pos];
        *buf = audio_queue[dequeue_pos];
        dequeue_pos = (dequeue_pos + 1) % QUEUE_DEPTH;
        uint32_t save = spin_lock_blocking(queue_spin_lock);
        queue_length--;
        queue_samples -= *len;
        spin_unlock(queue_spin_lock, save);
        return true;
    } 
    else return false;
}


/**
 * キューへサンプルを積む
 */
bool enqueue(int32_t* buf_in, uint32_t len)
{
	if (get_queue_length() < QUEUE_DEPTH)
	{
		audio_queue_len[enqueue_pos] = len;
		uint32_t i = len * 2;						// ステレオコピーのため2倍化;
		int32_t *audio_queue_p = audio_queue[enqueue_pos];
#if 1
		memmove( audio_queue_p, buf_in, i * (sizeof(int32_t)));
#else
		while( i-- )
		{
			*audio_queue_p++ = *buf_in++; 
		}
#endif
        enqueue_pos = (enqueue_pos + 1) % QUEUE_DEPTH;
        uint32_t save = spin_lock_blocking(queue_spin_lock);
        queue_length++;
		queue_samples += len;
        spin_unlock(queue_spin_lock, save);
		return true;
    }
	else return false;
}

/**
 * キューされた段数の取得
 */
uint32_t get_queue_length(void)
{
    uint32_t ret;
    uint32_t save = spin_lock_blocking(queue_spin_lock);
    ret = queue_length;
    spin_unlock(queue_spin_lock, save);
    return ret;
}

/**
 * キューされたサンプル総数の取得
 */
uint32_t get_queue_samples(void)
{
	uint32_t ret;
	uint32_t save = spin_lock_blocking(queue_spin_lock);
	ret = queue_samples;
	spin_unlock(queue_spin_lock, save);
	return ret;
}


// キューに積まれたオーバーサンプリング後のサンプル数を入力fsのサンプル数に換算する
// キューのサンプル総数はx2~x8オーバーサンプリング後で増量している。これをオーバーサンプリング比で換算する
uint32_t get_input_fs_referred_queue_samples(uint fs){
	uint32_t osr_shift;	// Over Sampling Ratio
	switch(fs) {
		case 384000:
		case 352800:	osr_shift = 0;	break;	// / 1
		case 192000:
		case 176400:	osr_shift = 1;	break;	// / 2
		case  96000:
		case  88200:	osr_shift = 2;	break;	// / 4
		case  48000:
		case  44100:
		default    :	osr_shift = 3;	break;	// / 8
	}
	return get_queue_samples() >> osr_shift;
}

// 以下未使用
#if 0

/**
 * 外部でキュー蓄積するためのポインタ取得
 */
bool get_enqueue_pointer(int32_t** buf)
{
	if (get_queue_length() < QUEUE_DEPTH){
		*buf = audio_queue[enqueue_pos];
		return true;
	} else {
		return false;
	}
}

/**
 * 外部でキュー蓄積したときの事後処理
 */
bool post_ext_enqueue_proc(uint32_t len)
{
	bool ret = false;
	if (get_queue_length() < QUEUE_DEPTH){
		audio_queue_len[enqueue_pos] = len;
		enqueue_pos = (enqueue_pos + 1) % QUEUE_DEPTH;
		uint32_t save = spin_lock_blocking(queue_spin_lock);
		queue_length++;
		spin_unlock(queue_spin_lock, save);
		ret = true;
	}
	return ret;
}
#endif