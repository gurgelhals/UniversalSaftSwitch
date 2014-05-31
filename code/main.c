/* 2014, Ralf Ramsauer
 * ralf@binary-kitchen.de
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <string.h>

#define ACT_ON (PORTB |= (1<<PB3))
#define ACT_OFF (PORTB &= ~(1<<PB3))

#define REL_ON (PORTB |= (1<<PB4))
#define REL_OFF (PORTB &= ~(1<<PB4))

#define PROG_ON (!(PIND & (1<<PD6)))

#define DE_RX (PORTD &= ~(1<<PD2))
#define DE_TX (PORTD |= (1<<PD2))

void rs485_send(const unsigned char *data, unsigned int length)
{
    cli();
    DE_TX;
    while (length--)
    {
        while (!(UCSRA & (1<<UDRE))); //wait...
        UDR = *data++;
    }
    while (!(UCSRA & (1<<UDRE))); //wait...
    DE_RX;
    sei();
}

unsigned char state;

#define RS485_IDLE 0
#define RS485_RXACT 1
#define RS485_DEVID 2
#define RS485_PAYLOAD 3

#define RS485_PREAMBLE 0x40
#define RS485_MYID 0xaa

unsigned char buffer[64];

ISR(USART_RX_vect)
{
    unsigned char in = UDR;

    static unsigned char state = RS485_IDLE;
    static unsigned char devid = 0;
    static unsigned char payload_length = 0;
    static unsigned char *ptr = buffer;

    if (state == RS485_IDLE && in == RS485_PREAMBLE) {
        state = RS485_RXACT;
    } else if (state == RS485_RXACT) {
        state = RS485_DEVID;
        devid = in;
    } else if (state == RS485_DEVID) {
        state = RS485_PAYLOAD;
        payload_length = in;
    } else if (state == RS485_PAYLOAD) {
        *ptr++ = in;
        payload_length--;

        if (payload_length == 0) {
            if (devid == RS485_MYID) {
                PORTB ^= (1<<PB3);
            }

            ptr = buffer;
            payload_length = 0;
            devid = 0;
            state = RS485_IDLE;
        }
    } else {
        ptr = buffer;
        payload_length = 0;
        devid = 0;
        state = RS485_IDLE;
    }

}

int main(void) {
    // disable all interrupts
    //cli();

    // PB3, PB4 as output
    DDRB = (1<<PB3)|(1<<PB4);

    // PD6 as input + pull up
    DDRD &= ~(1<<PD6);
    PORTD |= (1<<PD6);

    // PD2 (DE) as output and to Recv Mode
    DDRD |= (1<<PD2);
    DE_RX;

    // 38400bps @ 8MHz
    UBRRH = 0;
    UBRRL = 12;

    // Enable receiver and transmitter + enable receive interrupt
    UCSRB = (1<<RXEN)|(1<<TXEN)|(1<<RXCIE);

    // 8n1
    UCSRC = (1<<UCSZ1)|(1<<UCSZ0);

    _delay_ms(500);

    for(;;){
        if (PROG_ON) {
            ACT_ON;
            REL_ON;
        } else {
            ACT_OFF;
            REL_OFF;
        }
    }

    for(;;);
    return 0;
}
