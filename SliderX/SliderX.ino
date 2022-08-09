#define ENDSTOP_PIN A0

#define DIR_PIN 10
#define STEP_PIN 11
#define EN_PIN 12

#define LED_PIN 13

#define COMPARE_VALUE_TIMER OCR1A

#define TurnOnTimer1 (TIMSK1 |= (1 << OCIE1A))
#define TurnOffTimer1 (TIMSK1 &= ~(1 << OCIE1A))

#define COMMAND_PORT Serial

#define DEFAULT_JERK 1200000	 // mm/s3
#define DEFAULT_ACCELERATION 900 // mm/s2

#define DEFAULT_SPEED 30
#define MAX_SPEED 1000 // mm/s
#define HOMING_SPEED 20
#define BEGIN_SPEED 5

#define MAX_POSITION 600l

#define STEP_PER_MM 200 // pul/rev = 1000, vitme = 1605; 1000 / 5 = 200 pul/mm

#define SPEED_TO_CYCLE(x) (1000000.0 / (STEP_PER_MM * x))

#include "ScurveInterpolator.h"

String inputString;
bool stringComplete;

float DesireSpeed;
float OldSpeed;
float LinearSpeed;
float Jerk;
float Accel;
float DesirePosition;
float CurrentPosition;
long DesireSteps;
long PassedSteps;
unsigned long PassedTime;
long AccelSteps;
float TempCycle;

bool isEnding = false;
bool isHoming = false;
bool isMoving = false;
bool isLedOn = false;

const uint8_t mask[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
Scurve_Interpolator scurveAccel;

void setup()
{
	COMMAND_PORT.begin(115200);
	IOInit();
	setValue();
	TimerInit();
}

void loop()
{
	Home();
	SerialExecute();
	SliderExecute();
}

void IOInit()
{
	pinMode(ENDSTOP_PIN, INPUT_PULLUP);
	pinMode(DIR_PIN, OUTPUT);
	pinMode(STEP_PIN, OUTPUT);
	pinMode(EN_PIN, OUTPUT);
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(EN_PIN, 0);
}

void setValue()
{
	COMMAND_PORT.println("SliderX begin!");
	DesireSpeed = OldSpeed = DEFAULT_SPEED;
	Accel = DEFAULT_ACCELERATION;
	Jerk = DEFAULT_JERK;

	DesirePosition = 0;
	CurrentPosition = 0;
	DesireSteps = 0;
	PassedSteps = 0;

	scurveAccel.setMoveData(Accel, Jerk, DesireSpeed, BEGIN_SPEED, BEGIN_SPEED);
}

void TimerInit()
{
	noInterrupts();

	// Reset register relate to Timer 1
	// Reset register relate
	TCCR1A = TCCR1B = TCNT1 = 0;
	// Set CTC mode to Timer 1
	TCCR1B |= (1 << WGM12);
	// Set prescaler 1 to Timer 1
	TCCR1B |= (1 << CS10);
	// Normal port operation, OCxA disconnected
	TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0) | (1 << COM1B1) | (1 << COM1B0));

	interrupts();
}

void Home()
{
	if (isHoming && !digitalRead(ENDSTOP_PIN))
	{
		TurnOffTimer1;
		isHoming = false;
		isMoving = false;
		DesirePosition = 0;
		CurrentPosition = 0;
		DesireSteps = 0;
		PassedSteps = 0;
		AccelSteps = 0;
		COMMAND_PORT.println("Ok");
	}
}

void SliderExecute()
{

	if (DesirePosition == CurrentPosition)
		return;

	isMoving = true;
	DesireSteps = roundf(abs(DesirePosition - CurrentPosition) * STEP_PER_MM);

	CaculateLinearSpeed();
	FinishMoving();
	TempCycle = SPEED_TO_CYCLE(LinearSpeed);
	setIntCycle(TempCycle);
	TurnOnTimer1;
}

// intCycle us
void setIntCycle(float intCycle)
{
	int prescaler;

	if (intCycle > 4000)
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

	COMPARE_VALUE_TIMER = roundf(intCycle * 16 / prescaler) - 1;
	// COMPARE_VALUE_TIMER = roundf(intCycle * F_CPU / (1000000.0 * prescaler)) - 1;
}

ISR(TIMER1_COMPA_vect)
{
	if (isMoving)
	{
		if (PassedSteps == DesireSteps)
			return;

		fastDigitalWrite(STEP_PIN, 0);
		delayMicroseconds(3);
		fastDigitalWrite(STEP_PIN, 1);

		PassedSteps++;
		PassedTime += TempCycle;
		if (LinearSpeed < DesireSpeed && !isEnding)
			AccelSteps++;

		if (PassedSteps % 8 == 0)
		{
			isLedOn = !isLedOn;
			fastDigitalWrite(LED_BUILTIN, isLedOn);
		}
	}
}

void FinishMoving()
{
	if (isMoving && PassedSteps == DesireSteps)
	{
		CurrentPosition = DesirePosition;
		AccelSteps = 0;
		PassedTime = 0;
		LinearSpeed = 0;
		DesireSteps = 0;
		PassedSteps = 0;
		AccelSteps = 0;
		isMoving = false;
		isEnding = false;
		TurnOffTimer1;
		COMMAND_PORT.println("Ok");
	}
}

void CaculateLinearSpeed()
{
	if (DesireSteps - PassedSteps <= AccelSteps)
	{
		if (!isEnding)
		{
			isEnding = true;
			PassedTime = 0;
			if (DesireSpeed > LinearSpeed)
				DesireSpeed = LinearSpeed;
		}
		LinearSpeed = DesireSpeed - Accel * PassedTime / 1000000;
		return;
	}
	else if (LinearSpeed < DesireSpeed)
	{
		LinearSpeed = BEGIN_SPEED + Accel * PassedTime / 1000000;
	}
	else
	{
		LinearSpeed = DesireSpeed;
	}
}

void SerialExecute()
{
	while (COMMAND_PORT.available())
	{
		char inChar = (char)COMMAND_PORT.read();

		if (inChar == '\n')
		{
			stringComplete = true;
			break;
		}
		if (inChar != '\r')
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
		COMMAND_PORT.println(CurrentPosition);
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
		isHoming = true;
		DesireSpeed = HOMING_SPEED;
		DesirePosition = -1000;
	}

	if (messageBuffer == "M321")
	{
		DesireSpeed = inputString.substring(5).toFloat();
		// speed
		if (DesireSpeed < 0.01 && DesireSpeed > -0.01)
		{
			DesireSpeed = 0;
		}

		if (DesireSpeed != 0)
		{
			DesireSpeed = abs(DesireSpeed);
		}

		if (DesireSpeed > MAX_SPEED)
		{
			DesireSpeed = MAX_SPEED;
		}

		OldSpeed = DesireSpeed;
		COMMAND_PORT.println("Ok");
	}

	if (messageBuffer == "M322")
	{
		DesirePosition = inputString.substring(5).toFloat();

		if (DesirePosition < 0)
			DesirePosition = 0;
		if (DesirePosition > MAX_POSITION)
			DesirePosition = MAX_POSITION;
		if (DesirePosition == CurrentPosition)
		{
			COMMAND_PORT.println("Ok");
		}
		else
		{
			LinearSpeed = BEGIN_SPEED;
			TempCycle = DesireSpeed * SPEED_TO_CYCLE(DesireSpeed) / LinearSpeed;
			setIntCycle(TempCycle);
			DesireSpeed = OldSpeed;

			if (DesirePosition > CurrentPosition)
			{
				fastDigitalWrite(DIR_PIN, 1);
			}
			else
			{
				fastDigitalWrite(DIR_PIN, 0);
			}
		}
	}

	if (messageBuffer == "M323")
	{
		digitalWrite(EN_PIN, 1);
		TurnOffTimer1;
		COMMAND_PORT.println("Ok");
	}

	if (messageBuffer == "M324")
	{
		Accel = inputString.substring(5).toFloat();
		COMMAND_PORT.println("Ok");
	}

	inputString = "";
	stringComplete = false;
}

void fastDigitalWrite(uint8_t pin, uint8_t state)
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
