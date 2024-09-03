#ifndef _PINS_H_
#define _PINS_H_

bool get_group_48k(uint fs);
uint get_osr(uint fs);
float get_true_playback_fs(uint fs);
void volume(int32_t* buf, uint32_t sample_num, int32_t mul, uint shift);
void hbf_oversampler_reset(void);
void hbf_oversampler(int32_t** buf, uint *p_len, uint fs);
int32_t* get_dsp_buf_pointer(uint fs);
////void asrc(int32_t** buf, uint* p_len, uint32_t pitch); ////I2Sは使用しないため無効化
void dsp_reset(void);
void dsp_init(void);

#endif
