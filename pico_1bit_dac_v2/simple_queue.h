#ifndef _SIMPLE_QUEUE_H_
#define _SIMPLE_QUEUE_H_

void queue_reset(void);
void queue_init(void);
bool dequeue(int32_t** buff, uint32_t* len);
bool enqueue(int32_t* buff, uint32_t len);
uint32_t get_queue_length(void);
uint32_t get_queue_samples(void);
uint32_t get_input_fs_referred_queue_samples(uint fs);
//bool get_enqueue_pointer(int32_t** buff);
//bool post_ext_enqueue_proc(uint32_t len);


#define QUEUE_DEPTH		16							// キュー段数
#define QUEUE_TARGET_WATERLEVEL	(QUEUE_DEPTH - 4)	// FEEDBACK再生の目標レベル 
#define QUEUE_PLAY_THR	8							// 再生開始キュー段数
//#define QUEUE_WIDTH	(2 * (48 + 2) * 8)	// 2Ch x (1SOF Max @ 48k) x8 Over Sampling
#define QUEUE_WIDTH	(2 * (48 + 8) * 8)	// 2Ch x (1SOF Max @ 48k) x8 Over Sampling ※

/* USB サンプル超過受信・オーバーラン対策
・ホスト環境　Windows10 21H2
・課題
　OSのSpeakerプロパティでUSB Audio再生周波数を低い方向(192k->96k,96k->48k等)に切り替えた直後、
　初回再生時、21SOF目あたりでサンプル受信数が規定周波数の9％を超過することがある。
　これにより、バッファオーバーランが生じ再生NGとなる。
　現象は48kHz系への切り替えのみで発生。44kHz系への切り替えでは発生しない。
　OS依存の問題が疑われる。
  
  切替前[kHz]   ->切替後[kHz]　～20SOF  21SOF   USB規格許容 超過率
  ----------------------------------------------------------------
  192,176,96,88 ->  48         48spl    52spl   49spl       8.3%
  192,176       ->  96         96spl   105spl   97spl       9.3%
  192,176,96,88 ->  44         44~45    44~45   45spl       2.2% (規格内)
  192,176       ->  88         88~89    88~89   89spl       1.1% (規格内)

・暫定対策
　QUEUE_WIDTH(バッファの横幅)を48+2(+4%) から48+8(+16%)に増加した。
　実際の横幅は、ステレオ対応とx8 OverSampling係数を乗じて 2 * (48 +8) *8 としている。
　既知の不具合量を超える値を設定しているが、未知の不具合量発生時に備え、
  呼び出し側でも buf_len_limiter() を使い、サンプル処理数がQUEUE_WIDTHを超えないようにした。
*/

#endif
