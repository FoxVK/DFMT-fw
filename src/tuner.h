/* 
 * File:   tuner.h
 * Author: fox
 *
 * Created on 8. b?ezen 2015, 13:10
 */

#ifndef TUNER_H
#define TUNER_H
#include <inttypes.h>
#include "shared.h"

#ifdef	__cplusplus
extern "C" {
#endif

void tuner_init();
void tuner_write(int tuner_id, void* data, size_t len);
void tuner_read(int tuner_id, void* data, size_t len);
void tuner_hold_in_rst(int state);
Tuner_com_state tuner_com_state();
size_t tuner_rwed_bytes();

void tuner_audio_run(int tuner_id, int state);
size_t tuner_audio_get(const int tuner_id, void* buf, size_t max);
uint16_t * tuner_audio_setbuf(int tuner_id, uint16_t* buf);

#ifdef	__cplusplus
}
#endif

#endif	/* TUNER_H */

