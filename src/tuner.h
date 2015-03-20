/* 
 * File:   tuner.h
 * Author: fox
 *
 * Created on 8. b?ezen 2015, 13:10
 */

#ifndef TUNER_H
#define TUNER_H
#include <inttypes.h>

#ifdef	__cplusplus
extern "C" {
#endif

    typedef enum {
        TUNER_COM_IDLE, TUNER_COM_BUSY, TUNER_COM_WRONG_ADDR, TUNER_COM_WRITE_NAK, TUNER_COM_DEV_BUSY
    }Tuner_com_state;

    typedef struct{

        union{
            uint8_t byte;
            struct {
                char STCINT:1;       //Tune complete has been triggered.
                char reserved1_2:2;
                char RSQINT:1;       //Received Signal Quality measurement has been triggered.
                char reserved4_5:2;
                char ERR:1;          //Error
                char CTS:1;          //Clear to send next command.
            }bits;
        }STATUS;
        uint8_t ARGS[15];
    }Tuner_read_reply;

void tuner_init();
void tuner_write(int tuner_id, void* data, size_t len);
void tuner_read(int tuner_id, void* data, size_t len);
Tuner_com_state tuner_com_state();
size_t tuner_rwed_bytes();


#ifdef	__cplusplus
}
#endif

#endif	/* TUNER_H */

