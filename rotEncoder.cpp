
#include "rotEncoder.h"

RotEncoder::RotEncoder(byte swPin , byte clkPin , byte dtPin)
{
	_switchPin = swPin;
	_clkPin = clkPin;
	_dtPin = dtPin;
	pinMode(_switchPin, INPUT_PULLUP);

	pinMode(_clkPin, INPUT);
	pinMode(dtPin, INPUT);

	dtPinLast = digitalRead(dtPin);
}

uint16_t RotEncoder::pushCounter()
{

	if (!digitalRead(_switchPin) && !_swStatus) // currently switch is pushed and previosly not
	{ // This function is for identify user has pushed the button
		_swStatus = 1;  
		_timerSw = millis();
		
	}

	if (!digitalRead(_switchPin) && _swStatus && millis() - _timerSw > 10 && !swOn) // switch is in ON state
	{// This function is for deal with debounce error 
		_i++;
		//Serial.println(_i);
		
		_swStatus = 0;
		swOn = 1;
		counterResetTmr = millis();	

	}

	if (swOn && !digitalRead(_switchPin)) // switch is on status and currently is pushed also
	{ // this function is updating until switch pin being pushed
		_timerNc = millis() - _timerSw; // Time since first push
	}

	if (digitalRead(_switchPin) && swOn) // currently switch is not pushed and but in on state
	{
		//_timerNc = millis() - _timerSw;
		_timerSw = millis();
		
		swOn = 0;
		delay(2);
		//Serial.println("swoff");
		
	}
	/*
	if (millis() - counterResetTmr > 2000 || !resetCountVal)
	{
		_i = 0;
	}
	*/
	return _i;
}
//-----------------------------------
unsigned long RotEncoder::setTimerSw2Millis()
{
	_timerSw = millis();
	return _timerNc;
}

void RotEncoder::resetTimerNc()
{
	_timerNc = 0;
}

//-----------------------------------
void RotEncoder::pushCounterResetTimer(uint16_t resetTime)
{
	if (millis() - counterResetTmr > resetTime)
	{
		_i = 0;
	}
}

void RotEncoder::pushCounterReset()
{
	_i = 0;	
}

int8_t RotEncoder::pushTimerReset()
{
	_timerNc = 0;
}

unsigned long RotEncoder::pushTime()
{
	
	if (_timerNc == prev_timerNc )
	{
		_timerNc = 0;
	}
	
	prev_timerNc = _timerNc;
	
	return _timerNc;
}

int32_t RotEncoder::rotVal()
{
	dtVal = digitalRead(_dtPin);
	if (dtVal != dtPinLast)
	{
		if (digitalRead(_clkPin) != dtVal)
		{
			encoderPos--;
		}
		else
		{
			encoderPos++;
		}
		//Serial.println(encoderPos);
	}
	dtPinLast = dtVal;
	return encoderPos;
}

void RotEncoder::setEncoderPos(int32_t inVal)
{
	encoderPos = inVal;
}

int32_t RotEncoder::rotValBetween(int16_t inMin = 0, int16_t inMax = 18)
{
	int16_t outVal = (int16_t) rotVal();
	if(outVal < inMin)
	{
		setEncoderPos(inMax);
		outVal = inMax;
	}
	else if(outVal > inMax)
	{
		setEncoderPos(inMin);
		outVal = inMin;
	}
	return outVal;
}

void RotEncoder::pushCounterSet(int8_t inVal)
{
	_i = inVal;	
}

void RotEncoder::rotValSet(int16_t inVal)
{
	encoderPos = inVal;	
}
//RotEncoder RotEncoder;

