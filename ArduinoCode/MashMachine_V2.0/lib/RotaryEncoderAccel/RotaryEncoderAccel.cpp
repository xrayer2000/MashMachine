// -----
// RotaryEncoderAccel.cpp - Library for using rotary encoders.
// This class is implemented for use with the Arduino environment.
// Copyright (c) by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
// More information on: http://www.mathertel.de/Arduino
// -----
// 18.01.2014 created by Matthias Hertel
// -----

#include "Arduino.h"
#include "RotaryEncoderAccel.h"


// The array holds the values 1 for the entries where a position was decremented,
// a 1 for the entries where the position was incremented
// and 0 in all the other (no change or not valid) cases.

const int8_t KNOBDIR[] = {
  0, -1,  1,  0,
  1,  0,  0, -1,
  -1,  0,  0,  1,
0,  1, -1,  0  };


// positions: [3] 1 0 2 [3] 1 0 2 [3]
// [3] is the positions where my rotary switch detends
// ==> right, count up
// <== left,  count down


// ----- Initialization and Default Values -----

RotaryEncoderAccel::RotaryEncoderAccel(int pin1, int pin2) {

  // Remember Hardware Setup
  _pin1 = pin1;
  _pin2 = pin2;

  // Setup the input pins
  pinMode(pin1, INPUT);
  digitalWrite(pin1, HIGH);   // turn on pullup resistor

  pinMode(pin2, INPUT);
  digitalWrite(pin2, HIGH);   // turn on pullup resistor

  // when not started in motion, the current state of the encoder should be 3
  _oldState = 3;

  // start with position 0;
  _position = 0;
  _positionExt = 0;
  _positionExtPrev = 0;

  accel=0;
} // RotaryEncoderAccel()


int  RotaryEncoderAccel::getPosition() {
  return _positionExt;
}

void RotaryEncoderAccel::calculateDirection()
{

  if (_positionExtPrev > _positionExt) { 
    _positionExtPrev = _positionExt;
    _direction = Direction::COUNTERCLOCKWISE;
    //Serial.print(", Direction: COUNTERCLOCKWISE ");
  }  
  else if (_positionExtPrev < _positionExt) {  
    _positionExtPrev = _positionExt;
    _direction = Direction::CLOCKWISE;
    //Serial.print(", Direction: CLOCKWISE ");
  } 
  else{
    _positionExtPrev = _positionExt;
    _direction = Direction::NOROTATION;
  }
}

RotaryEncoderAccel::Direction RotaryEncoderAccel::getDirection()
{
  return _direction;
}

void RotaryEncoderAccel::setPosition(int newPosition)
{
  _position = ((newPosition << 2) | (_position & 0x01L));
  _positionExt = newPosition;
  _positionExtPrev = newPosition;
} // setPosition()

void RotaryEncoderAccel::tick(void)
{
  int sig1 = digitalRead(_pin1);
  int sig2 = digitalRead(_pin2);
  int8_t thisState = sig1 | (sig2 << 1);
  detectRotation = false;

  if (_oldState != thisState) {
    int oldPositionExt = _positionExt;
    _position += KNOBDIR[thisState | (_oldState<<2)];

    if (thisState == LATCHSTATE) {
      detectRotation = true;
      if (accel > 0) {
        unsigned long actualTick = millis();
        unsigned long delta = actualTick - prevTick;
        if (delta != 0 && delta < accel) {
          unsigned long increment = (accel / delta - 1) * KNOBDIR[thisState | (_oldState << 2)];
          _position += increment * multipleter;
        }
        prevTick = actualTick;
      }

      _positionExt = _position >> 2;
      // Only update time if position actually changed
      if (_positionExt != oldPositionExt) {
        _positionExtTimePrev = _positionExtTime;
        _positionExtTime = millis();
      }
    }

    _oldState = thisState;
  }
}

void RotaryEncoderAccel::setAccel(unsigned int value1, int value2 = 1) {
	prevTick = millis();
  accel = value1;
  multipleter = value2;
}

unsigned long RotaryEncoderAccel::getMillisBetweenRotations() const
{
  return (_positionExtTime - _positionExtTimePrev);
}

float RotaryEncoderAccel::getRPM()
{
  // calculate max of difference in time between last position changes or last change and now.
  unsigned long timeBetweenLastPositions = getMillisBetweenRotations();
  unsigned long timeToLastPosition = millis() - _positionExtTime;
  unsigned long t = max(timeBetweenLastPositions, timeToLastPosition);

  float rpm; 
  calculateDirection();

  if(_direction != Direction::NOROTATION)
  {
    // Serial.print(", Time: ");
    // Serial.print(_positionExtTime);
    // Serial.print(", TimePrev: ");
    // Serial.print(_positionExtTimePrev);
    // if(timeBetweenLastPositions == 0)
    // {
    //   Serial.println(" , timeBetweenLastPositions == 0");

    // }
    // else 
    // {
    //   Serial.println();
    // }

    switch (_direction) {
      case Direction::CLOCKWISE:
        rpm = 60000.0 / ((float)(t * 20 * 10));
        if(rpm < 1.0) rpm = 1;
          return rpm;
      case Direction::COUNTERCLOCKWISE:
        rpm = -60000.0 / ((float)(t * 20 * 10));
        if(rpm > -1.0) rpm = -1;
          return rpm;
    }  
    
  }
  else {
    rpm = 0;
  }
  
  return rpm;
}

// End
