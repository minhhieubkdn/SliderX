#define ENDSTOP_PIN A0

#define DIR_PIN 10
#define STEP_PIN 11
#define EN_PIN 12

#define LED_PIN 13

#define resume_accel_timer (TIMSK2 |= (1 << OCIE2A))
#define pause_accel_timer (TIMSK2 &= ~(1 << OCIE2A))

#define resume_pulse_timer (TIMSK1 |= (1 << OCIE1A))
#define pause_pulse_timer (TIMSK1 &= ~(1 << OCIE1A))

#define COMMAND_PORT Serial

#define DEFAULT_JERK 800000		 // mm/s3
#define DEFAULT_ACCELERATION 900 // mm/s2

#define DEFAULT_SPEED 50
#define MAX_SPEED 1000 // mm/s
#define HOMING_SPEED 20
#define BEGIN_SPEED 5

#define MAX_POSITION 500

#define DEFAULT_STEP_PER_MM 200 // pul/rev = 1000, vitme = 1605; 1000 / 5 = 200 pul/mm
#define SPEED_TO_PERIOD(x) (1000000.0 / (DEFAULT_STEP_PER_MM * x))
#define PERIOD_TO_SPEED(x) (1000000.0 / (DEFAULT_STEP_PER_MM * x))

#define ACCEL_EXECUTE_PERIOD 0.001 // second

#include <Arduino.h>
#include <ScurveInterpolator.h>

String inputString;
bool stringComplete;

float desired_speed;
float old_speed;
float jerk;
float accel;
float desired_position;
float current_position;
long total_desired_steps;
long current_steps;
float current_segment_pos;

bool is_homing = false;
bool is_enable_pulse_timer = false;
bool is_executing_M322 = false;

volatile bool led = false;
volatile unsigned long last_millis = 0;

volatile uint8_t timer2_loop_num = 1;
volatile uint8_t timer2_loop_index = 1;
volatile int last_period;
const uint8_t mask[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
Scurve_Interpolator scurve;

void setup()
{
	COMMAND_PORT.begin(115200);
	initIO();
	initValues();
	initTimers();
}

void loop()
{
	handle_serial_data();
	handle_slider_events();
}

void initIO()
{
	pinMode(ENDSTOP_PIN, INPUT_PULLUP);
	pinMode(DIR_PIN, OUTPUT);
	pinMode(STEP_PIN, OUTPUT);
	pinMode(EN_PIN, OUTPUT);
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(EN_PIN, 0);
}

void initValues()
{
	COMMAND_PORT.println("SliderX begin!");
	desired_speed = old_speed = DEFAULT_SPEED;
	accel = DEFAULT_ACCELERATION;
	jerk = DEFAULT_JERK;

	scurve.setMoveData(accel, jerk, desired_speed, BEGIN_SPEED, BEGIN_SPEED);
	scurve.setTimeTick(ACCEL_EXECUTE_PERIOD);

	desired_position = 0;
	current_position = 0;
	total_desired_steps = 0;
	current_steps = 0;
}

void initTimers()
{
	noInterrupts();

	TCCR1A = TCCR1B = TCNT1 = 0;
	TCCR1B |= (1 << WGM12);
	TCCR1B |= (1 << CS10);
	TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0) | (1 << COM1B1) | (1 << COM1B0));

	TCCR2A = TCCR2B = TCNT2 = 0;
	TCCR2A |= (1 << WGM21);
	TCCR2B |= (1 << CS20);

	interrupts();

	set_accel_timer_period(ACCEL_EXECUTE_PERIOD * 1000000);
}

// _period us
void set_accel_timer_period(int _period)
{
	int prescaler;
	if (_period < 16)
	{
		TCCR2B |= (1 << CS20);
		TCCR2B &= ~(1 << CS22);
		prescaler = 1;
		timer2_loop_num = 1;
		OCR2A = roundf(_period * 16 / prescaler - 1);
	}
	else if (_period < 1020)
	{
		TCCR2B |= (1 << CS22);
		TCCR2B &= ~(1 << CS20);
		prescaler = 64;
		timer2_loop_num = 1;
		OCR2A = roundf(_period * 16 / prescaler - 1);
	}
	else
	{
		TCCR2B |= (1 << CS22);
		TCCR2B &= ~(1 << CS20);
		prescaler = 64;
		timer2_loop_num = _period / 1000 + 1;
		OCR2A = roundf((_period / timer2_loop_num) * 16 / prescaler - 1);
	}
}

void set_pulse_timer_period(int _period)
{
	int prescaler;

	if (_period > 4000)
	{
		TCCR1B |= (1 << CS11);
		TCCR1B &= ~(1 << CS10);
		prescaler = 8;
	}
	else
	{
		TCCR1B &= ~(1 << CS11);
		TCCR1B |= (1 << CS10);
		prescaler = 1;
	}

	OCR1A = roundf(_period * 16 / prescaler) - 1;
	// COMPARE_VALUE_TIMER = roundf(_period * F_CPU / (1000000.0 * prescaler)) - 1;
}

void init_new_segment(float distance)
{
	if (distance > 0)
	{
		fast_digital_write(DIR_PIN, 1);
	}
	else
	{
		fast_digital_write(DIR_PIN, 0);
	}

	current_steps = 0;
	current_segment_pos = 0;
	total_desired_steps = abs(distance) * DEFAULT_STEP_PER_MM;

	scurve.setTarget(abs(distance));
	scurve.start();

	resume_accel_timer;
	is_enable_pulse_timer = true;
}

ISR(TIMER1_COMPA_vect)
{
	isr_generate_pulse();
}

ISR(TIMER2_COMPA_vect)
{
	if (timer2_loop_index == timer2_loop_num)
	{
		timer2_loop_index = 1;
		//
		isr_accel_execute();
	}
	else
	{
		timer2_loop_index++;
	}
}

void isr_accel_execute()
{
	is_enable_pulse_timer = false;
	pause_pulse_timer;
	if (scurve.update())
	{
		is_enable_pulse_timer = true;
		resume_pulse_timer;
	}
	else
	{
		float linear_step_num = (scurve.p - current_segment_pos) * DEFAULT_STEP_PER_MM;
		current_segment_pos = scurve.p;

		float linear_period = 1000000 * ACCEL_EXECUTE_PERIOD / linear_step_num;
		int int_period = roundf(linear_period);
		if (last_period != int_period)
		{
			set_pulse_timer_period(int_period);
			last_period = int_period;
		}

		is_enable_pulse_timer = true;
		resume_pulse_timer;
	}
}

void isr_generate_pulse()
{
	if (!is_enable_pulse_timer)
		return;

	fast_digital_write(STEP_PIN, 1);
	delayMicroseconds(5);
	fast_digital_write(STEP_PIN, 0);

	if (is_homing)
		return;

	current_steps++;
	if (current_steps >= total_desired_steps)
	{
		is_enable_pulse_timer = false;
		pause_pulse_timer;
		pause_accel_timer;
		COMMAND_PORT.println("Ok");
		is_executing_M322 = false;
		current_position = desired_position;
	}
}

void init_homing()
{
	fast_digital_write(DIR_PIN, 0);
	int homing_period = SPEED_TO_PERIOD(HOMING_SPEED);
	set_pulse_timer_period(homing_period);

	is_homing = true;
	is_enable_pulse_timer = true;
	resume_pulse_timer;
}

void handle_slider_events()
{
	if (is_homing)
	{
		blink_led();
		if (!digitalRead(ENDSTOP_PIN))
		{
			is_enable_pulse_timer = false;
			pause_pulse_timer;
			is_homing = false;
			current_position = 0;
			current_steps = 0;
			COMMAND_PORT.println("OkHoming");
		}
	}
	else if (is_executing_M322)
	{
		blink_led();
	}
}

void blink_led()
{
	if (millis() - last_millis > 100)
	{
		last_millis = millis();
		led = !led;
		fast_digital_write(LED_BUILTIN, led);
	}
}

void handle_serial_data()
{
	while (COMMAND_PORT.available())
	{
		char inChar = (char)COMMAND_PORT.read();

		if (inChar == '\n')
		{
			stringComplete = true;
			break;
		}
		else if (inChar != '\r')
			inputString += inChar;
	}

	if (!stringComplete)
		return;

	if (inputString == "IsSliderX")
	{
		COMMAND_PORT.println("YesSliderX");
		inputString = "";
		stringComplete = false;
		return;
	}
	else if (inputString == "Position")
	{
		COMMAND_PORT.println(current_position);
		inputString = "";
		stringComplete = false;
		return;
	}
	else if (inputString == "HTs")
	{
		COMMAND_PORT.println(digitalRead(ENDSTOP_PIN));
		inputString = "";
		stringComplete = false;
		return;
	}

	String messageBuffer = inputString.substring(0, 4);

	if (messageBuffer == "M320")
	{
		init_homing();
	}
	else if (messageBuffer == "M321")
	{
		desired_speed = inputString.substring(5).toFloat();
		// speed
		if (desired_speed < 0.01 && desired_speed > -0.01)
		{
			desired_speed = 0;
		}

		if (desired_speed != 0)
		{
			desired_speed = abs(desired_speed);
			scurve.setMaxVel(desired_speed);
		}

		if (desired_speed > MAX_SPEED)
		{
			desired_speed = MAX_SPEED;
		}

		old_speed = desired_speed;
		COMMAND_PORT.println("Ok");
	}
	else if (messageBuffer == "M322")
	{
		if (!is_executing_M322)
		{
			desired_position = inputString.substring(5).toFloat();

			if (desired_position < 0)
				desired_position = 0;
			else if (desired_position > MAX_POSITION)
				desired_position = MAX_POSITION;
			else if (desired_position == current_position)
			{
				COMMAND_PORT.println("Ok");
			}
			else
			{
				float distance = desired_position - current_position;
				init_new_segment(distance);
				is_executing_M322 = true;
			}
		}
	}
	else if (messageBuffer == "M323")
	{
		digitalWrite(EN_PIN, 1);
		pause_accel_timer;
		COMMAND_PORT.println("Ok");
	}
	else if (messageBuffer == "M324")
	{
		accel = inputString.substring(5).toFloat();
		scurve.setMaxAcc(accel);
		COMMAND_PORT.println("Ok");
	}
	else if (messageBuffer == "M325")
	{
		jerk = inputString.substring(5).toFloat();
		scurve.setMaxJerk(jerk);
		COMMAND_PORT.println("Ok");
	}
	else if (messageBuffer == "M326")
	{
		float ve = inputString.substring(5).toFloat();
		scurve.setVelStart(ve);
		scurve.setVelEnd(ve);
		COMMAND_PORT.println("Ok");
	}

	inputString = "";
	stringComplete = false;
}

void fast_digital_write(uint8_t pin, uint8_t state)
{

	if (pin < 8)
	{
		state ? PORTD |= mask[pin] : PORTD &= ~mask[pin];
	}
	else if (pin < 14)
	{
		pin -= 8;
		state ? PORTB |= mask[pin] : PORTB &= ~mask[pin];
	}
	else if (pin < 22)
	{
		pin -= 14;
		state ? PORTC |= mask[pin] : PORTC &= ~mask[pin];
	}
}
