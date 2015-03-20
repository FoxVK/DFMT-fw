#include <xc.h>
#include <sys/attribs.h>
#include <inttypes.h>
#include "tuner.h"

#define TUNER_A_W 0b00100010 //SEN pin = 0
#define TUNER_B_W 0b11000110 //SEN pin = 1
#define TUNER_A_R 0b00100011 //SEN pin = 0
#define TUNER_B_R 0b11000111 //SEN pin = 1

 #define debughalt() __asm__ volatile (" sdbbp 0")

static volatile enum  {
    COM_IDLE,
    COM_START, COM_ADDRSND, COM_ADDRACK,
    COM_WR, COM_ACKING,
    COM_STOPPING, COM_STOPPED
}com_state;

static volatile enum  {
    COM_DIR_NONE, COM_DIR_READING, COM_DIR_WRITING
}com_dir;

static volatile Tuner_com_state last_err = TUNER_COM_IDLE;

static volatile uint8_t  *data_buf;
static volatile int      ptr = 0;
static volatile size_t   max_len;

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
    {
        Nop();
        IFS1bits.I2C1BIF=0;
        debughalt();
    }

    if(com_state == COM_IDLE)
        debughalt();

    switch (com_state)
    {
        case COM_START:
            com_state = COM_ADDRSND;
            I2C1TRN = addr;
            break;

        case COM_ADDRSND:
            if(I2C1STATbits.ACKSTAT == 1)
            {
                I2C1CONbits.PEN = 1;
                com_state = COM_STOPPED;
                last_err = TUNER_COM_WRONG_ADDR;
                break;
            }
            else
                Nop();
        case COM_WR:
            if(com_dir == COM_DIR_WRITING)
            {
                if(I2C1STATbits.ACKSTAT == 1)
                {
                    I2C1CONbits.PEN = 1; //stop
                    com_state = COM_STOPPED;
                    last_err = TUNER_COM_WRITE_NAK;
                    break;
                }

                if(ptr<max_len)
                {
                    I2C1TRN = data_buf[ptr];
                    ptr++;
                    com_state = COM_WR;
                }
                else
                {
                    I2C1CONbits.PEN = 1; //stop
                    com_state = COM_STOPPED;
                }
            }
            else
            {
                I2C1CONbits.RCEN = 1; //allow receiving
                com_state = COM_ACKING;
            }
            break;

        case COM_ACKING:
            if(ptr<max_len)
            {
                data_buf[ptr] = I2C1RCV;

                if((data_buf[0]&&0x80)==0 && ptr==0)
                {   //CTS==0 we have to try it again later
                    I2C1CONbits.ACKDT = 1; //NAK
                    I2C1CONbits.ACKEN = 1;
                    com_state = COM_STOPPING;
                    last_err = TUNER_COM_DEV_BUSY;
                    break;
                }

                ptr++;
                
            }

            if(ptr<max_len)
            {
                I2C1CONbits.ACKDT = 0; //ACK
                com_state = COM_WR;
            }
            else
            {
                I2C1CONbits.ACKDT = 1; //NAK
                com_state = COM_STOPPING;
            }
            I2C1CONbits.ACKEN = 1;

            break;

        case COM_STOPPING:
            I2C1CONbits.PEN = 1;
            com_state = COM_STOPPED;
            break;

        case COM_STOPPED:
            //I2C1CONbits.ON = 0;
            //IEC1bits.I2C1MIE = 0;
            com_state = COM_IDLE;
            if(last_err == TUNER_COM_BUSY)
                last_err = TUNER_COM_IDLE;
            break;

    }
    IFS1bits.I2C1MIF = 0;
}

void tuner_rw_start(void* data, size_t len)
{
    data_buf = (uint8_t*)data;
    max_len = len;
    ptr = 0;
    last_err = TUNER_COM_BUSY;

    I2C1CONbits.ON = 0;

    IFS1bits.I2C1MIF = 0;
    IEC1bits.I2C1MIE = 1;
    I2C1CONbits.ON = 1;
    Nop(); Nop();
}

void tuner_write(int tuner_id, void* data, size_t len)
{
    tuner_rw_start(data, len);
    addr = tuner_id == 1 ? TUNER_B_W : TUNER_A_W;
    com_dir = COM_DIR_WRITING;
    com_state = COM_START;
    I2C1CONbits.SEN = 1; //start condition, wait for interrupt

}

void tuner_read(int tuner_id, void* data, size_t len)
{
    tuner_rw_start(data, len);
    addr = tuner_id == 1 ? TUNER_B_R : TUNER_A_R;
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
    int i;
    //TODO dokonfigurovat pocatecni hodnotu na low
    PORTBbits.RB13 = 0; //tuners reset active
    for(i=0; i<1000; i++)Nop(); //delay
    PORTBbits.RB13 = 1; //tuners reset inactive
    for(i=0; i<1000; i++)Nop(); //delay

    //timer5 init
    IPC5SET = 0x1F; //maximal interrupt priority
    //T5CONbits.TCKPS = 0b011; //1:8
    PR5 = 610; //2*32.768kHz
    TMR5 = 0;

    IFS0bits.T5IF = 0;
    IEC0bits.T5IE = 1;
    T5CONbits.ON = 1;

    //I2C init
    TRISBbits.TRISB8 = 1;
    TRISBbits.TRISB9 = 1;
    I2C1BRG = /*0x88B;*/0x0C2; //DS 24 str 19: PBclk = 40MHz clk = 100kHz
    I2C1CONbits.DISSLW = 0; //because errata
    
    IPC8bits.I2C1IP = 5; //i2c Interrupt priority 5
    
    IFS1bits.I2C1MIF = 0;    
    IEC1bits.I2C1MIE = 1;
    
    IFS1bits.I2C1BIF = 0;
    IEC1bits.I2C1BIE = 0;

    com_dir = COM_DIR_NONE;
    com_state = COM_IDLE;

    volatile char cmd_pwrup[] = {0x01, 0b00000001, 0x05};

   


    tuner_write(0, &cmd_pwrup, sizeof(cmd_pwrup));
    while(tuner_com_state() != TUNER_COM_IDLE)
            Nop();
     Nop();
    
    volatile static  Tuner_read_reply * st;
    st = &cmd_pwrup;

    for(i=0; i<10000;i++)
        Nop();


    while(1){
        tuner_read(0, &cmd_pwrup, sizeof(cmd_pwrup));
        while(tuner_com_state() == TUNER_COM_BUSY)
            Nop();

        if(tuner_com_state() != TUNER_COM_DEV_BUSY)
            break;
        for(i=0; i<1000;i++)
            Nop();
    }
    ///////////////////////////////////////////
    char tune_helax[] = {0x20, 0x00, 0x24, 0x9a, 0x00};

    tuner_write(0, &tune_helax, sizeof(tune_helax));
    while(tuner_com_state() != TUNER_COM_IDLE)
            Nop();
     Nop();


    for(i=0; i<10000;i++)
        Nop();


    while(1){
        tuner_read(0, &cmd_pwrup, 1);
        while(tuner_com_state() == TUNER_COM_BUSY)
            Nop();

        if(tuner_com_state() != TUNER_COM_DEV_BUSY)
            break;
        for(i=0; i<1000;i++)
            Nop();
    }

    ///////////////////////////////////////////
     //debughalt();

    U1TXREG = 'd';
    //while(1)
        Nop();
}

Tuner_com_state tuner_com_state()
{
    return last_err;
}

size_t tuner_rwed_bytes()
{
    return ptr+1;
}


void __ISR (_TIMER_5_VECTOR, IPL7AUTO) Timer5Handler (void)
{
    //generates ~32.768 kHz clock on RB7
    PORTBINV = 1<<7;
    IFS0bits.T5IF=0; // Be sure to clear the Timer5 interrupt status
}