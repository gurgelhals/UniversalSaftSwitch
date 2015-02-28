/* 2014-2015, Ralf Ramsauer
 * ralf@binary-kitchen.de
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
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

static volatile unsigned char buffer[MAX_PAYLOAD];

static volatile unsigned char rs485_myid;

static volatile unsigned char state = RS485_IDLE;
static volatile unsigned char src = 0;
static volatile unsigned char dst = 0;
static volatile unsigned char payload_length = 0;
static volatile unsigned char *ptr = buffer;
static volatile unsigned char crc = 0;

static volatile uint8_t uss = 0;

uint8_t ee_rs485_myid EEMEM = 0xaa; // default device id
uint8_t ee_relais_inverted EEMEM = 0;

static unsigned char relais_inverted;

const unsigned char crc8_table[256] PROGMEM = {
 0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
 0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
 0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
 0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
 0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
 0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
 0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
 0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
 0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
 0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
 0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
 0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
 0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
 0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
 0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
 0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
};

#define UPDATE_CRC crc = pgm_read_byte(crc8_table + (crc ^ in))

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

    crc = 0;
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
        UPDATE_CRC;
        state = RS485_RXACT;
    } else if (state == RS485_RXACT) {
        UPDATE_CRC;
        state = RS485_SRC;
        src = in;
    } else if (state == RS485_SRC) {
        UPDATE_CRC;
        state = RS485_DST;
        dst = in;
    } else if (state == RS485_DST) {
        UPDATE_CRC;
        state = RS485_PAYLOAD_LEN;
        my_payload_length = payload_length = in;
        if (payload_length > MAX_PAYLOAD) {
            goto reset;
        } else if (payload_length == 0) {
            state = RS485_PAYLOAD;
        }
    } else if (state == RS485_PAYLOAD_LEN) {
        UPDATE_CRC;
        *ptr++ = in;
        my_payload_length--;

        if (my_payload_length == 0) {
            state = RS485_PAYLOAD;
        }
    } else if (state == RS485_PAYLOAD) {
        TIMER_OFF;


        if (crc == in) {
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
    crc = 0;
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
    
    crc = 0;

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
				ACT_ON;
                            } else {
                                REL_OFF;
				ACT_OFF;
                            }
                            ackval = RS485_SUCCESS;
                        }
                        break;
                    case USS_CMD_ON:
                        if (payload_length == 1) {
                            if (relais_inverted) {
                                REL_OFF;
				ACT_OFF;
                            } else {
                                REL_ON;
				ACT_ON;
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
