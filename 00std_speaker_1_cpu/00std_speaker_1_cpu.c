#include "nrf52833.h"


#define max_its 200
//s

void wait(int reps) {
    volatile uint32_t a;
    for (a=0; a<reps;a++);
}

int main(void) {
    // loop variables
    int i,it = 0;
    // set speaker pin as output
    NRF_P0->PIN_CNF[0] = 0x00000003; // P0.00

    while(1) {
        if (it == max_its) break;
        for (i=1;i<2000;i++){
            NRF_P0->OUTSET = (0x00000001 << 0); // speaker pin high
            wait(i);
            NRF_P0->OUTCLR = (0x00000001 << 0); // speaker pin low
            wait(i);
        }
        ++it;
    }

    
}
