// UART4 Library
// Yasasvi Vanapalli

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    40MHz

// Hardware configuration:
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "uart4.h"
#include "stringf.h"

// Port C mask
#define UART4_TX_MASK 32    // PC5
#define UART4_RX_MASK 16    // PC4

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize UART4
void initUart4()
{
    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R4;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;
    _delay_cycles(3);

    // Configure UART4 pins
    GPIO_PORTC_DR2R_R |= UART4_TX_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTC_DEN_R |= UART4_TX_MASK | UART4_RX_MASK;    // enable digital on UART0 pins
    GPIO_PORTC_AFSEL_R |= UART4_TX_MASK | UART4_RX_MASK;  // use peripheral to drive PA0, PA1
    GPIO_PORTC_PCTL_R &= ~(GPIO_PCTL_PC5_M | GPIO_PCTL_PC4_M); // clear bits 0-7
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U4TX | GPIO_PCTL_PC4_U4RX;
                                                        // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART4 to 250k baud (assuming fcyc = 40 MHz), 8N1 format
    UART4_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART4_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
    UART4_IBRD_R = 10;                                  // r = 40 MHz / (Nx9600Hz), set floor(r)=260, where N=16
    UART4_FBRD_R = 0;                                  // round(fract(r)*64)=27
    UART4_LCRH_R = UART_LCRH_WLEN_8;                    // configure for 8N1 w/ 16-level FIFO
    UART4_LCRH_R |= UART_LCRH_STP2;                     // set stop bits to be 2
    UART4_CTL_R |= UART_CTL_UARTEN;    // enable TX, RX, and module
    UART4_IM_R |= UART_IM_RXIM;                         // set the receive interrupt mask
    //UART4_RIS_R |= UART_RIS_RXRIS;                      //
    NVIC_EN1_R |= 1 << (INT_UART4-16-32);                   // turn-on interrupt 76 (UART4) in NVIC
}

// Set baud rate as function of instruction cycle frequency
void setUart4BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
                                                        // where r = fcyc / 16 * baudRate
    divisorTimes128 += 1;                               // add 1/128 to allow rounding
    UART4_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART4_IBRD_R = divisorTimes128 >> 7;                // set integer value to floor(r)
    UART4_FBRD_R = ((divisorTimes128) >> 1) & 63;       // set fractional value to round(fract(r)*64)
    UART4_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2;    // configure for 8N1 w/ 16-level FIFO
    UART4_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // turn-on UART0
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart4(char c)
{
    while (UART4_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART4_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart4(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart4(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart4()
{
    while (UART4_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART4_DR_R & 0xFF;                        // get character from fifo
}


//
void getsUart4(USER_DATA* data)
{
    uint8_t count = 0;
    //uint8_t number = (uint8_t);

    while(true)
    {
        char input = getcUart4();
        uint8_t x = (uint8_t) input;
        if(x == 13)
        {
            data->buffer[count] = '\0';
            return;
        }
        if(count == MAX_CHARS)
        {
            data->buffer[count + 1] = '\0';
            break;
        }
        if((x == 8 || x == 127) && count>0)
        {
            count -= 1;
        }
        if(x>=32)
        {
            data->buffer[count]=input;
            count+=1;
        }
    }
}

// Returns the status of the receive buffer
bool kbhitUart4()
{
    return !(UART4_FR_R & UART_FR_RXFE);
}


