/* 2014, Ralf Ramsauer
 * ralf@binary-kitchen.de
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
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

#define TIMER_OFF {\
    TIMSK = 0;\
    TCNT1 = 0;\
    TCCR1B = 0;\
    }

#define TIMER_ON {\
    TIMSK |= (1<<OCIE1A);\
    TIFR |= (1<<OCF1A);\
    TCNT1 = 0;\
    TCCR1B = (1<<CS10);\
    }

#define RS485_SUCCESS 0
#define RS485_FAIL 1

#define MAX_PAYLOAD 64

#define RS485_PREAMBLE 0x40

#define RS485_IDLE 0
#define RS485_RXACT 1
#define RS485_SRC 2
#define RS485_DST 3
#define RS485_PAYLOAD_LEN 4
#define RS485_PAYLOAD 5
#define RS485_CRC 6

#define USS_CMD_ON     0x81
#define USS_CMD_OFF    0x82

#define USS_CMD_SETID  0x83
#define USS_CMD_INVERT 0x84

static unsigned char buffer[MAX_PAYLOAD];

static volatile unsigned char rs485_myid;

static volatile unsigned char state = RS485_IDLE;
static volatile unsigned char src = 0;
static volatile unsigned char dst = 0;
static volatile unsigned char payload_length = 0;
static volatile unsigned char *ptr = buffer;

static volatile uint8_t uss = 0;

uint8_t ee_rs485_myid EEMEM = 0xaa; // default device id
uint8_t ee_relais_inverted EEMEM = 0;

static unsigned char relais_inverted;

uint8_t rs485_send(const uint8_t *data, unsigned int length)
{
    uint8_t retval = RS485_FAIL;

    UCSRB &= ~(1<<RXCIE);

    DE_TX;

    while(length--)
    {
        UCSRA |= (1<<TXC);
        while (!(UCSRA & (1<<UDRE))); //wait...
        UDR = *data;
        while (!(UCSRA & (1<<TXC))); // wait until bits are sent

        if ( UCSRA & (1<<RXC) )
        {
            unsigned char in = UDR;
            if (in != *data) {
                // Oh, collision. Damn...
                _delay_us(100);
                goto out;
            }
        } else {
            // Didn't receive anything. This actually should never happen.
            _delay_us(100);
            goto out;
        }

        data++;
    }

    retval = RS485_SUCCESS;

out:
    DE_RX;
    UCSRB |= (1<<RXCIE);
    return retval;
}

uint8_t rs485_send_frame(const uint8_t src, const uint8_t dst, const uint8_t *data, unsigned int length)
{
    uint8_t *ptr = buffer;

    unsigned char sum = length + 5;

    *ptr++ = RS485_PREAMBLE;
    *ptr++ = src;
    *ptr++ = dst;
    *ptr++ = length;
    while (length--)
    {
        *ptr++ = *data++;
    }

    *ptr++ = 0xaa;

    return rs485_send(buffer, sum);
}

ISR(TIMER1_COMPA_vect)
{
    TCNT1 = 0;

    state = RS485_IDLE;

    const static unsigned char f = 'f';
    rs485_send(&f, 1);

    TIMER_OFF;
}

ISR(USART_RX_vect)
{
    unsigned char in = UDR;

    static uint8_t my_payload_length = 0;

    TCNT1 = 0;
    TIMER_ON;

    if (state == RS485_IDLE && in == RS485_PREAMBLE) {
        state = RS485_RXACT;
    } else if (state == RS485_RXACT) {
        state = RS485_SRC;
        src = in;
    } else if (state == RS485_SRC) {
        state = RS485_DST;
        dst = in;
    } else if (state == RS485_DST) {
        state = RS485_PAYLOAD_LEN;
        my_payload_length = payload_length = in;
        if (payload_length > MAX_PAYLOAD) {
            goto reset;
        } else if (payload_length == 0) {
            state = RS485_PAYLOAD;
        }
    } else if (state == RS485_PAYLOAD_LEN) {
        *ptr++ = in;
        my_payload_length--;

        if (my_payload_length == 0) {
            state = RS485_PAYLOAD;
        }
    } else if (state == RS485_PAYLOAD) {
        TIMER_OFF;

        uint8_t crc = in;
        if (in == 0xaa) {
            if (dst == rs485_myid) {
                uss = 1;
            }
        }
        goto reset;
    }

    return;

reset:
    ptr = buffer;
    my_payload_length = 0;
    state = RS485_IDLE;
}

int main(void) {
    // disable all interrupts
    cli();

    // PB3, PB4 as output
    DDRB = (1<<PB3)|(1<<PB4);

    // PD6 as input + pull up
    DDRD &= ~(1<<PD6);
    PORTD |= (1<<PD6);

    // PD2 (DE) as output and to Recv Mode
    DDRD |= (1<<PD2);
    DE_RX;

    // 115200bps @ 14.7456 MHz
    UBRRH = 0;
    UBRRL = 7;

    // Enable receiver and transmitter + enable receive interrupt
    UCSRB = (1<<RXEN)|(1<<TXEN)|(1<<RXCIE);

    // 8n1
    UCSRC = (1<<UCSZ1)|(1<<UCSZ0);

    // Configure Timer
    TCCR1A = 0;
    TCCR1B = 0; // Timer deactivated

    TCNT1 = 0;
    OCR1A = 1475; // ~100.029µS

    // Disable interrupt
    TIMER_OFF;

    rs485_myid = eeprom_read_byte(&ee_rs485_myid);
    relais_inverted = eeprom_read_byte(&ee_relais_inverted);

    _delay_ms(500);

    sei();

    for(;;) {
        if (uss)
        {
            // We have to access the buffer, so disable all interrupts to make sure that no one else will access it
            cli();

            uint8_t ackval = RS485_FAIL;

            if (payload_length == 0) {
                // error.
            } else {
                uint8_t cmd = buffer[0];

                switch (cmd) {
                    case USS_CMD_INVERT:
                        if (payload_length == 2) {
                            if (buffer[1] && relais_inverted == 0) {
                                relais_inverted = 1;
                                eeprom_write_byte(&ee_relais_inverted, 1);
                            } else if (buffer[1] == 0 && relais_inverted) {
                                relais_inverted = 0;
                                eeprom_write_byte(&ee_relais_inverted, 0);
                            }
                            ackval = RS485_SUCCESS;
                        }
                        break;
                    case USS_CMD_SETID:
                        if (payload_length == 2) {
                            if (buffer[1] != rs485_myid) {
                                rs485_myid = buffer[1];
                                eeprom_write_byte(&ee_rs485_myid, rs485_myid);
                            }
                            ackval = RS485_SUCCESS;
                        }
                        break;

                    case USS_CMD_OFF:
                        if (payload_length == 1) {
                            if (relais_inverted) {
                                REL_ON;
                            } else {
                                REL_OFF;
                            }
                            ackval = RS485_SUCCESS;
                        }
                        break;
                    case USS_CMD_ON:
                        if (payload_length == 1) {
                            if (relais_inverted) {
                                REL_OFF;
                            } else {
                                REL_ON;
                            }
                            ackval = RS485_SUCCESS;
                        }
                        break;

                    default:
                        break;
                }
            }

            rs485_send_frame(rs485_myid, src, &ackval, 1);

            uss = 0;
            sei();
        }
    }

    return 0;
}
