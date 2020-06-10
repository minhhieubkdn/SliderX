#define ENDSTOP_PIN A0

#define DIR_PIN 10
#define STEP_PIN 11
#define EN_PIN 12

#define LED_PIN 13

#define COMPARE_VALUE_TIMER OCR1A

#define TurnOnTimer1 (TIMSK1 |= (1 << OCIE1A))
#define TurnOffTimer1 (TIMSK1 &= ~(1 << OCIE1A))

#define COMMAND_PORT Serial

#define MAX_SPEED 300
#define DEFAULT_SPEED 30
#define HOMING_SPEED (DEFAULT_SPEED / 2)

#define STEP_PER_MM 240

#define SPEED_TO_CYCLE(x) (1000000.0 / (STEP_PER_MM * x))

#include "MultiThread.h"

String inputString;
bool stringComplete;
float DesireSpeed;
float DesirePosition;
float CurrentPosition;
long DesireStepPosition;
long CurrentStepPosition;

bool PRESSED_LOGIC = LOW;
bool isHoming = false;
bool blink = false;

MultiThread LedBlinkScheduler;

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
	LedBlink();
}

void IOInit()
{
	pinMode(DIR_PIN, OUTPUT);
	pinMode(STEP_PIN, OUTPUT);
	pinMode(EN_PIN, OUTPUT);
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(EN_PIN, 1);
}

void setValue()
{
	COMMAND_PORT.println("Begin:");
	DesireSpeed = DEFAULT_SPEED;

	DesirePosition = 0;
	CurrentPosition = 0;
	DesireStepPosition = 0;
	CurrentStepPosition = 0;
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
	//Normal port operation, OCxA disconnected
	TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0) | (1 << COM1B1) | (1 << COM1B0));

	interrupts();
}

void LedBlink()
{
	if (!blink)
	{
		digitalWrite(LED_BUILTIN, 0);
		return;
	}

	RUN_EVERY(LedBlinkScheduler, COMPARE_VALUE_TIMER / 16);
	digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void Home()
{
	if (isHoming && !digitalRead(ENDSTOP_PIN))
	{
		TurnOffTimer1;
		isHoming = false;
		DesireSpeed = DEFAULT_SPEED;
		DesirePosition = 0;
		CurrentPosition = 0;
		DesireStepPosition = 0;
		CurrentStepPosition = 0;
		digitalWrite(EN_PIN, 0);
		COMMAND_PORT.println("Ok");
	}
}

void SliderExecute()
{
	//speed
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
	//position
	if (DesirePosition == CurrentPosition)
		return;

	DesireStepPosition = roundf(DesirePosition * STEP_PER_MM);

	if (DesirePosition - CurrentPosition > 0)
	{
		digitalWrite(DIR_PIN, 1);
	}
	else if (DesirePosition - CurrentPosition < 0)
	{
		digitalWrite(DIR_PIN, 0);
	}

	setIntCycle(SPEED_TO_CYCLE(DesireSpeed));
	digitalWrite(EN_PIN, 0);
	TurnOnTimer1;
}

//intCycle us
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

	COMPARE_VALUE_TIMER = roundf(intCycle * F_CPU / (1000000.0 * prescaler)) - 1;
}

ISR(TIMER1_COMPA_vect)
{
	if (DesireStepPosition > CurrentStepPosition)
	{
		CurrentStepPosition++;
	}
	else if (DesireStepPosition < CurrentStepPosition)
	{
		CurrentStepPosition--;
	}
	else
	{
		CurrentPosition = DesirePosition;
		TurnOffTimer1;
		blink = false;
		COMMAND_PORT.println("Ok");
		return;
	}

	digitalWrite(STEP_PIN, 0);
	delayMicroseconds(2);
	digitalWrite(STEP_PIN, 1);
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

		inputString += inChar;
	}

	if (!stringComplete)
		return;
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
		COMMAND_PORT.println("Ok");
	}

	if (messageBuffer == "M322")
	{
		DesirePosition = inputString.substring(5).toFloat();
		blink = true;
	}

	if (messageBuffer == "M323")
	{
		digitalWrite(EN_PIN, 1);
		TurnOffTimer1;
		COMMAND_PORT.println("Ok");
	}

	inputString = "";
	stringComplete = false;
}
