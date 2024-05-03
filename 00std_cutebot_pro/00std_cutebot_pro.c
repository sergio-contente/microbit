
#include <stdio.h>
#include <nrf.h>
#include "nrf52833.h"


#define STOPMOTORS 0
#define FORWARD 1
#define BACKWARD 2
#define TURNLEFT 3
#define TURNRIGHT 4

#define LEFTLEDRED 5
#define RIGHTLEDRED 6

#define LEFTLEDGREEN 7
#define RIGHTLEDGREEN 8

#define LEFTLEDBLUE 9
#define RIGHTLEDBLUE 10

#define LEFTLEDWHITE 11
#define RIGHTLEDWHITE 12

#define RIGHTLEDOFF 13
#define LEFTLEDOFF 14

#define NONE 15

static uint8_t pdu[8+1] = { 0 };
// static uint8_t pdu[] = {
//     0x00, // header
//        1, // length
//     0xE
// };

/*
sources:
    https://github.com/elecfreaks/pxt-Cutebot-Pro/blob/master/main.ts
I2C:
    same a Cutebot
*/

/*
motor control
[
    0x99,
    command,   // 0x01; set, 0x09: stop
    motor,     // 0x01: left, 0x02: right, 0x03 both
    direction, // 0x01: forward, 0x00: backward
    speed,     // between 0 and 100
    0x00,
    0x88,
]
*/
void delayc(uint32_t time);

#define MOTOR_SPEED 50 // [0...100]
uint8_t I2CBUF_MOTOR_LEFT_FWD[]   = {0x99,0x01,0x01,0x01,MOTOR_SPEED,0x00,0x88};
uint8_t I2CBUF_MOTOR_LEFT_BACK[]  = {0x99,0x01,0x01,0x00,MOTOR_SPEED,0x00,0x88};

uint8_t I2CBUF_MOTOR_RIGHT_FWD[]  = {0x99,0x01,0x02,0x01,MOTOR_SPEED,0x00,0x88};
uint8_t I2CBUF_MOTOR_RIGHT_BACK[] = {0x99,0x01,0x02,0x00,MOTOR_SPEED,0x00,0x88};

uint8_t I2CBUF_MOTORS_STOP[]      = {0x99,0x09,0x03,0x00,0x00,0x00,0x88};

/*
LED control
[
    0x99,
    0x0f,
    led,       // 0x02: left, 0x01: right, 0x03 both
    r,
    g,
    b,
    0x88,
]
*/
#define LED_INTENSITY 0xff // [0x00...0xff]
uint8_t I2CBUF_LED_LEFT_WHITE[]   = {0x99,0x0f,0x02,LED_INTENSITY,LED_INTENSITY,LED_INTENSITY,0x88};
uint8_t I2CBUF_LED_LEFT_RED[]     = {0x99,0x0f,0x02,LED_INTENSITY,         0x00,         0x00,0x88};
uint8_t I2CBUF_LED_LEFT_GREEN[]   = {0x99,0x0f,0x02,         0x00,LED_INTENSITY,         0x00,0x88};
uint8_t I2CBUF_LED_LEFT_BLUE[]    = {0x99,0x0f,0x02,         0x00,         0x00,LED_INTENSITY,0x88};
uint8_t I2CBUF_LED_LEFT_OFF[]     = {0x99,0x0f,0x02,         0x00,         0x00,         0x00,0x88};
uint8_t I2CBUF_LED_RIGHT_WHITE[]  = {0x99,0x0f,0x01,LED_INTENSITY,LED_INTENSITY,LED_INTENSITY,0x88};
uint8_t I2CBUF_LED_RIGHT_RED[]    = {0x99,0x0f,0x01,LED_INTENSITY,         0x00,         0x00,0x88};
uint8_t I2CBUF_LED_RIGHT_GREEN[]  = {0x99,0x0f,0x01,         0x00,LED_INTENSITY,         0x00,0x88};
uint8_t I2CBUF_LED_RIGHT_BLUE[]   = {0x99,0x0f,0x01,         0x00,         0x00,LED_INTENSITY,0x88};
uint8_t I2CBUF_LED_RIGHT_OFF[]    = {0x99,0x0f,0x01,         0x00,         0x00,         0x00,0x88};

void delayc(uint32_t time) {
    volatile uint32_t a;
    for (a=0; a< time; a++); //;a++);
}

void i2c_init(void) {
   //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... ...A A: DIR:   0=Input
    // .... .... .... .... .... .... .... ..B. B: INPUT: 1=Disconnect
    // .... .... .... .... .... .... .... CC.. C: PULL:  0=Disabled
    // .... .... .... .... .... .DDD .... .... D: DRIVE: 6=S0D1
    // .... .... .... ..EE .... .... .... .... E: SENSE: 0=Disabled
    // xxxx xxxx xxxx xx00 xxxx x110 xxxx 0010 
    //    0    0    0    0    0    6    0    2 0x00000602
    NRF_P0->PIN_CNF[26]           = 0x00000602; // SCL (P0.26)
    NRF_P1->PIN_CNF[0]            = 0x00000602; // SDA (P1.00)

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .... AAAA A: ENABLE: 5=Enabled
    // xxxx xxxx xxxx xxxx xxxx xxxx xxxx 0101 
    //    0    0    0    0    0    0    0    5 0x00000005
    NRF_TWI0->ENABLE              = 0x00000005;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... ...A AAAA A: PIN:    26 (P0.26)
    // .... .... .... .... .... .... ..B. .... B: PORT:    0 (P0.26)
    // C... .... .... .... .... .... .... .... C: CONNECT: 0=Connected
    // 0xxx xxxx xxxx xxxx xxxx xxxx xx01 1010 
    //    0    0    0    0    0    0    1    a 0x0000001a
    NRF_TWI0->PSEL.SCL            = 0x0000001a;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... ...A AAAA A: PIN:    00 (P1.00)
    // .... .... .... .... .... .... ..B. .... B: PORT:    1 (P1.00)
    // C... .... .... .... .... .... .... .... C: CONNECT: 0=Connected
    // 0xxx xxxx xxxx xxxx xxxx xxxx xx10 0000 
    //    0    0    0    0    0    0    2    0 0x00000020
    NRF_TWI0->PSEL.SDA            = 0x00000020;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // AAAA AAAA AAAA AAAA AAAA AAAA AAAA AAAA A: FREQUENCY: 0x01980000==K100==100 kbps
    NRF_TWI0->FREQUENCY           = 0x01980000;

    //  3           2            1           0
    // 1098 7654 3210 9876 5432 1098 7654 3210
    // .... .... .... .... .... .... .AAA AAAA A: ADDRESS: 16
    // xxxx xxxx xxxx xxxx xxxx xxxx x001 0000 
    //    0    0    0    0    0    0    1    0 0x00000010
    NRF_TWI0->ADDRESS             = 0x10;
}

void i2c_send(uint8_t* buf, uint8_t buflen) {
    uint8_t i;

    i=0;
    NRF_TWI0->TXD                 = buf[i];
    NRF_TWI0->EVENTS_TXDSENT      = 0;
    NRF_TWI0->TASKS_STARTTX       = 1;
    i++;
    while(i<buflen) {
        while(NRF_TWI0->EVENTS_TXDSENT==0);
        NRF_TWI0->EVENTS_TXDSENT  = 0;
        NRF_TWI0->TXD             = buf[i];
        i++;
    }
    while(NRF_TWI0->EVENTS_TXDSENT==0);
    NRF_TWI0->TASKS_STOP     = 1;
}

void TurnLeft(){
    i2c_send(I2CBUF_MOTOR_RIGHT_BACK,   sizeof(I2CBUF_MOTOR_RIGHT_BACK));
    i2c_send(I2CBUF_MOTOR_LEFT_FWD,    sizeof(I2CBUF_MOTOR_LEFT_FWD));
}

void TurnRight(){
    i2c_send(I2CBUF_MOTOR_RIGHT_FWD,  sizeof(I2CBUF_MOTOR_RIGHT_BACK));
    i2c_send(I2CBUF_MOTOR_LEFT_BACK,   sizeof(I2CBUF_MOTOR_LEFT_BACK));
}

void goStraight(){
    i2c_send(I2CBUF_MOTOR_RIGHT_FWD,   sizeof(I2CBUF_MOTOR_RIGHT_FWD));
    i2c_send(I2CBUF_MOTOR_LEFT_FWD,    sizeof(I2CBUF_MOTOR_LEFT_FWD));
}

void goBack(){
    i2c_send(I2CBUF_MOTOR_RIGHT_BACK,  sizeof(I2CBUF_MOTOR_RIGHT_BACK));
    i2c_send(I2CBUF_MOTOR_LEFT_BACK,   sizeof(I2CBUF_MOTOR_LEFT_BACK));
}

void stopMotors(){
        i2c_send(I2CBUF_MOTORS_STOP,  sizeof(I2CBUF_MOTORS_STOP));
}

void leftRedOn(){
    i2c_send(I2CBUF_LED_LEFT_RED,      sizeof(I2CBUF_LED_LEFT_RED));
}
void leftGreenOn(){
    i2c_send(I2CBUF_LED_LEFT_GREEN,      sizeof(I2CBUF_LED_LEFT_GREEN));
}
void leftBlueOn(){
    i2c_send(I2CBUF_LED_LEFT_BLUE,      sizeof(I2CBUF_LED_LEFT_BLUE));
}
void leftWhiteOn(){
    i2c_send(I2CBUF_LED_LEFT_WHITE,    sizeof(I2CBUF_LED_LEFT_WHITE));
}
void leftOff(){
    i2c_send(I2CBUF_LED_LEFT_OFF,      sizeof(I2CBUF_LED_LEFT_OFF));
}

void rightRedOn(){
    i2c_send(I2CBUF_LED_RIGHT_RED,      sizeof(I2CBUF_LED_RIGHT_RED));
}
void rightGreenOn(){
    i2c_send(I2CBUF_LED_RIGHT_GREEN,      sizeof(I2CBUF_LED_RIGHT_GREEN));
}
void rightBlueOn(){
    i2c_send(I2CBUF_LED_RIGHT_BLUE,      sizeof(I2CBUF_LED_RIGHT_BLUE));
}
void rightWhiteOn(){
    i2c_send(I2CBUF_LED_RIGHT_WHITE,    sizeof(I2CBUF_LED_RIGHT_WHITE));
}
void rightOff(){
    i2c_send(I2CBUF_LED_RIGHT_OFF,      sizeof(I2CBUF_LED_RIGHT_OFF));
}


void init_rx(void){

    
    // confiureg HF clock
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {}

    // configure radio
    NRF_RADIO->MODE          = (  RADIO_MODE_MODE_Ble_LR125Kbit << RADIO_MODE_MODE_Pos);
    NRF_RADIO->TXPOWER       = (  RADIO_TXPOWER_TXPOWER_Pos8dBm << RADIO_TXPOWER_TXPOWER_Pos);
    NRF_RADIO->PCNF0         = (                              8 << RADIO_PCNF0_LFLEN_Pos)          |
                               (                              1 << RADIO_PCNF0_S0LEN_Pos)          |
                               (                              0 << RADIO_PCNF0_S1LEN_Pos)          |
                               (                              2 << RADIO_PCNF0_CILEN_Pos)          |
                               (     RADIO_PCNF0_PLEN_LongRange << RADIO_PCNF0_PLEN_Pos)           |
                               (                              3 << RADIO_PCNF0_TERMLEN_Pos);
    NRF_RADIO->PCNF1         = (                    sizeof(pdu) << RADIO_PCNF1_MAXLEN_Pos)         |
                               (                              0 << RADIO_PCNF1_STATLEN_Pos)        |
                               (                              3 << RADIO_PCNF1_BALEN_Pos)          |
                               (      RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos)         |
                               (   RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos);
    NRF_RADIO->BASE0         = 0xAAAAAAAAUL;
    NRF_RADIO->TXADDRESS     = 0UL;
    NRF_RADIO->RXADDRESSES   = (RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos);
    NRF_RADIO->TIFS          = 0;
    NRF_RADIO->CRCCNF        = (         RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos)           |
                               (     RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
    NRF_RADIO->CRCINIT       = 0xFFFFUL;
    NRF_RADIO->CRCPOLY       = 0x00065b; // CRC poly: x^16 + x^12^x^5 + 1
    NRF_RADIO->FREQUENCY     = 10;
    NRF_RADIO->PACKETPTR     = (uint32_t)pdu;

    // receive
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |
                        (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos) |
                        (RADIO_SHORTS_DISABLED_RXEN_Enabled << RADIO_SHORTS_DISABLED_RXEN_Pos);
    NRF_RADIO->TASKS_RXEN    = 1;

    NRF_RADIO->INTENCLR = 0xffffffff;
    NVIC_EnableIRQ(RADIO_IRQn);
    NRF_RADIO->INTENSET = (RADIO_INTENSET_DISABLED_Enabled << RADIO_INTENSET_DISABLED_Pos);

}

void RADIO_IRQHandler(void) {
    if (NRF_RADIO->EVENTS_DISABLED) {
        NRF_RADIO->EVENTS_DISABLED = 0;

        if (NRF_RADIO->CRCSTATUS != RADIO_CRCSTATUS_CRCSTATUS_CRCOk) {
            puts("Invalid CRC");
        } else {
            printf("Received packet (%dB): %s\n", pdu[1], &pdu[2]);
        }
    }
}

int main(void) {

    int command = NONE;
    i2c_init();
    init_rx();
    while(1) {
        __WFE();
        command = (int) &pdu[2];
        // if signal available: receive signal and change variable command

        switch (command){
            // motors
            case FORWARD:
                goStraight();
                break;
            case BACKWARD:
                goBack();
                break;
            case TURNLEFT:
                TurnLeft();
                break;
            case TURNRIGHT:
                TurnRight();
                break;
            case STOPMOTORS:
                stopMotors();
                break;

            // left red
            case LEFTLEDRED:
                leftRedOn();
                break;
            case LEFTLEDBLUE:
                leftBlueOn();
                break;
            case LEFTLEDGREEN:
                leftGreenOn();
                break;
            case LEFTLEDWHITE:
                leftWhiteOn();
                break;
            case LEFTLEDOFF:
                leftOff();
                break;

            // right led
            case RIGHTLEDRED:
                rightRedOn();
                break;
            case RIGHTLEDBLUE:
                rightBlueOn();
                break;
            case RIGHTLEDGREEN:
                rightGreenOn();
                break;
            case RIGHTLEDWHITE:
                rightWhiteOn();
                break;
            case RIGHTLEDOFF:
                rightOff();
                break;
            //None
            default:
                break;
        }
    }   
}
