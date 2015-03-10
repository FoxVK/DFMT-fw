#include <xc.h>
#include <sys/attribs.h>
#include <inttypes.h>
#include "tuner.h"

#define TUNER_A_W 0b00100010 //SEN pin = 0
#define TUNER_B_W 0b11000110 //SEN pin = 1
#define TUNER_A_R 0b00100011 //SEN pin = 0
#define TUNER_B_R 0b11000111 //SEN pin = 1

static enum  {
    COM_IDLE,
    COM_START, COM_ADDRSND, COM_ADDRACK,
    COM_WR, COM_ACKING,
    COM_STOPPING, COM_STOPPED
}com_state;

static enum  {
    COM_DIR_NONE, COM_DIR_READING, COM_DIR_WRITING
}com_dir;

static uint8_t  *data_buf;
static int      ptr = 0;
static size_t   max_len;

static uint8_t addr = 0;

/*
Turn on the I 2 C module by setting the ON bit (I2CxCON<15>) to ?1?.
Assert a Start condition on SDAx and SCLx.
Send the I 2 C device address byte to the slave with a write indication.
Wait for and verify an Acknowledge from the slave.
Send the serial memory address high byte to the slave.
Wait for and verify an Acknowledge from the slave.
Send the serial memory address low byte to the slave.
Wait for and verify an Acknowledge from the slave.
Assert a Repeated Start condition on SDAx and SCLx.
Send the device address byte to the slave with a read indication.
Wait for and verify an Acknowledge from the slave.
Enable master reception to receive serial memory data.
Generate an ACK or NACK condition at the end of a received byte of data.
Generate a Stop condition on SDAx and SCLx.
 */

void __ISR (_I2C_1_VECTOR, IPL5AUTO) I2C1Handler (void)
{

    if(IFS1bits.I2C1BIF)
        Nop();

    if(com_state == COM_IDLE)
        Nop();

    switch (com_state)
    {
        case COM_START:
            com_state = COM_ADDRSND;
            I2C1TRN = addr;
            break;

        case COM_ADDRSND:
            if(I2C1STATbits.ACKSTAT == 0)
            {
                com_state = COM_STOPPING;
                break;
            }
        case COM_WR:
            if(com_dir == COM_DIR_WRITING)
            {
                if(I2C1STATbits.ACKSTAT == 0) //TODO ucesat
                {
                    com_state = COM_STOPPING;
                    break;
                }

                if(ptr<max_len)
                {
                    I2C1TRN = data_buf[ptr];
                    ptr++;
                    com_state = COM_WR;
                }
                else
                    com_state = COM_STOPPING;
            }
            else
            {
                I2C1CONbits.RCEN = 1; //allow receiving
                com_state = COM_ACKING;
            }


        case COM_ACKING:
            if(ptr<max_len)
            {
                data_buf[ptr] = I2C1RCV;
                ptr++;
            }

            if(ptr<max_len)
            {
                I2C1CONbits.ACKDT = 1;
                com_state = COM_WR;
            }
            else
            {
                I2C1CONbits.ACKDT = 0; //NAK
                com_state = COM_STOPPING;
            }
            I2C1CONbits.ACKEN = 1;

            break;

        case COM_STOPPING:
            I2C1CONbits.PEN = 1;
            com_state = COM_STOPPED;
            break;

        case COM_STOPPED:
            com_state = COM_IDLE;
            break;

    }


    IFS1bits.I2C1MIF = 0;
}

void tuner_rw_start(void* data, size_t len)
{
    data_buf = (uint8_t*)data;
    max_len = len;
    ptr = 0;

    I2C1CONbits.ON = 0;
    I2C1CONbits.ON = 1;
    Nop();
}

void tuner_write(int tuner_id, void* data, size_t len)
{
    tuner_rw_start(data, len);
    addr = tuner_id ? TUNER_B_W : TUNER_A_W;
    com_dir = COM_DIR_WRITING;
    com_state = COM_START;
    I2C1CONbits.SEN = 1; //start condition, wait for interrupt

}

void tuner_read(int tuner_id, void* data, size_t len)
{
    tuner_rw_start(data, len);
    addr = tuner_id ? TUNER_B_R : TUNER_A_R;
    com_dir = COM_DIR_READING;
    com_state = COM_START;
    I2C1CONbits.SEN = 1; //start condition, wait for interrupt
}

int tuner_bytes_received()
{
    if(com_state == COM_IDLE)
        return ptr;
    else
        return -1;
}

void tuner_init()
{
    //timer5 init
    IPC5SET = 0x1F; //maximal interrupt priority
    //T5CONbits.TCKPS = 0b011; //1:8
    PR5 = 610; //2*32.768kHz
    TMR5 = 0;

    IFS0bits.T5IF = 0;
    IEC0bits.T5IE = 1;
    T5CONbits.ON = 1;

    //I2C init
    I2C1BRG = 0x0C2; //DS 24 str 19: PBclk = 40MHz clk = 100kHz
    
    IPC8bits.I2C1IP = 5; //Interrupt priority 5
    
    IFS1bits.I2C1MIF = 0;    
    IEC1bits.I2C1MIE = 1;
    
    IFS1bits.I2C1BIF = 0;
    IEC1bits.I2C1BIE = 1;

    com_dir = COM_DIR_NONE;
    com_state = COM_IDLE;
}



