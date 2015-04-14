/* 
 * File:   shared.h
 * Author: fox
 *
 * Created on 26. b?ezen 2015, 20:14
 */

#ifndef SHARED_H
#define	SHARED_H

#include <inttypes.h>

#define USB_VID 0x00a6
#define USB_PID 0xff01

#define TUNNEL_INTERFACE_ID 2
#define TUNNEL_EP_IN (0x02 | (1<<7))
#define TUNNEL_EP_OUT 0x02

#define TUNNEL_TUNER_SELECT_MASK 1
#define TUNNEL_RW_SELECT_MASK 2
#define TUNNEL_PING_MASK 4
#define TUNNEL_ERROR_MASK 128

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

#endif	/* SHARED_H */

