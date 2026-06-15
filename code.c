#define F_CPU 1000000UL  //1MHz MCU clock

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

/* -------------------------------------------------
   
   ADC0808 CONNECTIONS

   PB0 = ADC clock (Timer0 OC0)
   PB1 = ALE
   PB5 = START
   PB3 = EOC input

   PC0-PC7 = ADC data bus

   PE0-PE2 = channel select

   OE tied directly to VCC

-------------------------------------------------- */

#define ALE   PB1
#define START PB5
#define EOC   PB3

/* Servo positions for 1 MHz clock */
#define SERVO_POS_0     125   /* ~1.00 ms */
#define SERVO_POS_45    155   /* ~1.24 ms */
#define SERVO_POS_90    188   /* ~1.50 ms */
#define SERVO_POS_135   220   /* ~1.76 ms */

/* ------------------------------------------------- */

/* ADC CLOCK */

void timer0_adc_clock(void)
{
    DDRB |= (1 << PB0);

    TCCR0 =
        (1 << WGM01) |	  // clear timer on compare mode
        (1 << COM00) |
        (1 << CS00);

    /* 20 kHz ADC clock */
    OCR0 = 24;
}
// 0 -> ocr0 -> reset -> repeat

/* ------------------------------------------------- */

/* SERVO PWM */
// PWM mode

void timer1_servo_init(void)
{
    DDRD |= (1 << PD5);  //PD5 signal to servo

    TCCR1A =
        (1 << COM1A1) |
        (1 << WGM11);

    TCCR1B =
        (1 << WGM13) |	//fast PWM
        (1 << WGM12) |
        (1 << CS11);

    /* 20 ms period */
    ICR1 = 2499;

    /* start centered initially */
    OCR1A = SERVO_POS_90;
}

/* ------------------------------------------------- */

/* ADC INIT */

void adc_init(void)
{
    /* ADC data bus input (digital val input to MCU) */
    DDRC = 0x00;
    PORTC = 0x00; //disable pullup resistors

    /* Channel select output */
    DDRE = 0x07;
    PORTE = 0x00; //disable pullup resistors

    /* Control outputs */
    DDRB |= (1 << ALE) | (1 << START);

    /* EOC input */
    DDRB &= ~(1 << EOC);

    /* Initial LOW */
    PORTB &= ~((1 << ALE) | (1 << START));

    _delay_ms(10);
}

/* ------------------------------------------------- */

/* ADC READ */
// read one analog signal at a time

uint8_t adc_read(uint8_t channel)
{
    uint8_t value;

    /* Select channel */
    PORTE = (channel & 0x07);

    _delay_us(20);

    /* ALE HIGH */
    PORTB |= (1 << ALE);

    /* START HIGH */
    PORTB |= (1 << START);

    _delay_us(20);

    /* ALE LOW */
    PORTB &= ~(1 << ALE);

    /* START LOW */
    PORTB &= ~(1 << START);

    /* Wait EOC LOW */
    while (PINB & (1 << EOC));  //conversion to digital value

    /* Wait EOC HIGH */
    while (!(PINB & (1 << EOC)));  // wait for conversion to finish

    /* setting delay to ensure data is available */
    _delay_us(50);

    /* read ADC output */
    value = PINC;

    return value;
}

/* ------------------------------------------------- */

/* AVERAGED ADC */
// reduces noise by averaging multiple readings, smoother movement

uint8_t read_avg(uint8_t channel)
{
    uint16_t sum = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        sum += adc_read(channel);
    }

    return (uint8_t)(sum / 8);
}

/* ------------------------------------------------- */

/* SERVO CONTROL */
// sets PWM pulse width

void servo_set_pulse(uint16_t pulse)
{
    if (pulse < 125)  // prevent smaller pulse
        pulse = 125;

    if (pulse > 250)  // prevents larger pulse
        pulse = 250;

    OCR1A = pulse;  // change pulse width (timer automatically generates PWM) 
}


void servo_set_position(uint8_t pos)
{
// 0,1,2,3 can be used instead of servo positions

    switch (pos)
    {
        case 0:
            servo_set_pulse(SERVO_POS_0);
            break;

        case 1:
            servo_set_pulse(SERVO_POS_45);
            break;

        case 2:
            servo_set_pulse(SERVO_POS_90);
            break;

        case 3:
            servo_set_pulse(SERVO_POS_135);
            break;
    }
}

/* ------------------------------------------------- */

/* MAIN */

int main(void)
{
	uint8_t l1, l2, l3, l4;  // store LDR values

	uint8_t current_pos = 2;  // center initially
	uint8_t candidate_pos;	// new position

	uint8_t threshold = 25; // to ignore small changes
	uint8_t stable_count = 0;  // used to confirm stability

	uint8_t max_value;

	timer0_adc_clock();
	timer1_servo_init();
	adc_init();

	servo_set_position(current_pos);

	while (1)
	{
		/* read all LDRs */
		l1 = read_avg(0);
		l2 = read_avg(1);
		l3 = read_avg(2);
		l4 = read_avg(3);

		/* assume LDR1 brightest initially */
		max_value = l1;	// brightest light
		candidate_pos = 0;  // direction of brightest light

		/* compare LDR2 */
		if (l2 > max_value)
		{
			max_value = l2;
			candidate_pos = 1;
		}

		/* compare LDR3 */
		if (l3 > max_value)
		{
			max_value = l3;
			candidate_pos = 2;
		}

		/* compare LDR4 */
		if (l4 > max_value)
		{
			max_value = l4;
			candidate_pos = 3;
		}

		
		/* ignore small fluctuations i.e. check if movement is needed */

		if ((candidate_pos != current_pos) &&
		(
		((max_value - l1) < threshold) &&
		((max_value - l2) < threshold) &&
		((max_value - l3) < threshold) &&
		((max_value - l4) < threshold)
		))
		{
		  candidate_pos = current_pos;
		}


		/* stability check */

		if (candidate_pos != current_pos)
		{
			stable_count++;

			if (stable_count >= 3)  // only change pos if 3 stable counts
			{
				current_pos = candidate_pos;

				servo_set_position(current_pos);

				stable_count = 0;  //reset count
			}
		}
		else
		{
			stable_count = 0;
		}

		_delay_ms(50);
	}
}
