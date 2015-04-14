#include <xc.h>
#include <sys/attribs.h>
#include <inttypes.h>
#include "tuner.h"

#define TUNER_A_W 0b00100010 //SEN pin = 0
#define TUNER_B_W 0b11000110 //SEN pin = 1
#define TUNER_A_R 0b00100011 //SEN pin = 0
#define TUNER_B_R 0b11000111 //SEN pin = 1

#define I2S_BUF_SIZE (2*2*48)

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


static volatile struct  {
    uint8_t buf[I2S_BUF_SIZE];
    size_t head;
    size_t count;  //count of unreaden bytes
}  audio_data[2];



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


void __ISR (_I2C_1_VECTOR, IPL3AUTO) I2C1Handler (void)
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

void __ISR (_SPI_1_VECTOR, IPL5AUTO) I2S1Handler (void)
{
    //debughalt();
    static int limit = 0;
    while(!SPI1STATbits.SPIRBE)
    {
        static volatile unsigned int s ;
        s = SPI1BUF;

        audio_data[0].buf[audio_data[0].head] = (uint8_t)s;
        audio_data[0].count++;
        audio_data[0].head++;
        Nop();

        if(audio_data[0].head >= I2S_BUF_SIZE)
            audio_data[0].head = 0;

        if(audio_data[0].count>= I2S_BUF_SIZE)
            audio_data[0].count = I2S_BUF_SIZE;

        if(limit<200)
            limit++;
        else
            Nop();
    }
    IFS1bits.SPI1RXIF = 0;
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

static void timer_rclk_init()
{
    //timer5 init
    //IPC5SET = 0x1F; //maximal interrupt priority
    IPC5bits.T5IP = 7; //prioriy 1-7 0=disabled
    IPC5bits.T5IS = 3; //sub prio 0-3
    //T5CONbits.TCKPS = 0b011; //1:8
    PR5 = 610; //2*32.768kHz
    TMR5 = 0;

    IFS0bits.T5IF = 0;
    IEC0bits.T5IE = 1;
    T5CONbits.ON = 1;
}

static void i2c_init()
{
    //I2C init
    TRISBbits.TRISB8 = 1;
    TRISBbits.TRISB9 = 1;
    I2C1BRG = /*0x88B;*/0x0C2; //DS 24 str 19: PBclk = 40MHz clk = 100kHz
    I2C1CONbits.DISSLW = 0; //because errata

    IPC8bits.I2C1IP = 2; //prioriy 1-7 0=disabled

    IFS1bits.I2C1MIF = 0;
    IEC1bits.I2C1MIE = 1;

    IFS1bits.I2C1BIF = 0;
    IEC1bits.I2C1BIE = 0;

    com_dir = COM_DIR_NONE;
    com_state = COM_IDLE;
}

static void i2s_a_init()
{
    SPI1CONbits.ON = 0;
    Nop();
    SPI1CONbits.DISSDO = 1;     //dissable output bit
    SPI1CONbits.ENHBUF = 1;     //enchanced buffer enable
    SPI1CONbits.MSTEN = 1;      //enable master - we will generate clock
    SPI1CON2bits.AUDEN = 1;     //enable audio
    SPI1CONbits.SRXISEL = 2;    //interrupt is generated whe buffer is half full
    SPI1CONbits.CKP = 1;        //clock idle in low

    while(!SPI1STATbits.SPIRBE)
    {
        int flush = SPI1BUF;
    }
    SPI1STATbits.SPIROV = 0;    //no overflow
    
    SPI1BRG = 12;       //clock,  32bit frame X 48khz sampling = 1536kHz - now we have 1538,46

    IPC7bits.SPI1IP = 5; //prioriy 1-7 0=disabled
    IPC7bits.SPI1IS = 0; //subprio 0-3
    
    IEC1bits.SPI1RXIE = 1; //enable Rx interrupt
}

//Start or stop capturing I2S audio
void tuner_audio_run(int tuner_id, int state)
{
    audio_data[tuner_id].head = 0;
    audio_data[tuner_id].count = 0;

    if(tuner_id == 0)
    {
        IFS1CLR = 0b111 << 4;
        SPI1CONbits.ON = state;
    }
    else
    {
        IFS1CLR = 0b111 << 18;
        SPI2CONbits.ON = state;
    }        
}

size_t tuner_audio_get(const int tuner_id, void*buf, size_t max)
{
    uint8_t * b = buf;

    if(tuner_id)
    {
        if(!SPI2CONbits.ON)
            return 0;
    }
    else
    {
        if(!SPI1CONbits.ON)
            return 0;
    }
    //begin critical section
    if(tuner_id)
        IEC1bits.SPI2RXIE = 0;
    else
        IEC1bits.SPI1RXIE = 0;
    
        int ptr = audio_data[tuner_id].head;
        int count = audio_data[tuner_id].count;

        if (count>max)
            count = max;

        audio_data[tuner_id].count -= count;

    if(tuner_id)
        IEC1bits.SPI2RXIE = 1;
    else
        IEC1bits.SPI1RXIE = 1;
    //end critical section

    ptr -= count;
    if(ptr<0)
        ptr += I2S_BUF_SIZE;

    int i;
    for(i=0; i<count; i++)
    {
        b[i]=audio_data[tuner_id].buf[ptr];
        ptr++;
        if(ptr>=I2S_BUF_SIZE)
            ptr=0;
    }

    return count;
}

void tuner_hold_in_rst(int state)
{
    PORTBbits.RB13 = state ? 0 : 1;
}

void tuner_init()
{
    int i;
    //TODO dokonfigurovat pocatecni hodnotu na low
    tuner_hold_in_rst(1); //tuners reset active
    for(i=0; i<1000; i++)Nop(); //delay
    tuner_hold_in_rst(0); //tuners reset inactive
    for(i=0; i<1000; i++)Nop(); //delay

    timer_rclk_init();

    i2c_init();

    i2s_a_init();
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