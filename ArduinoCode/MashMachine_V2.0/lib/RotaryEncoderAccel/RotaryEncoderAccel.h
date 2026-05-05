// -----
// RotaryEncoderAccel.h - Library for using rotary encoders.
// This class is implemented for use with the Arduino environment.
// Copyright (c) by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
// More information on: http://www.mathertel.de/Arduino
// -----
// 18.01.2014 created by Matthias Hertel
// -----

#ifndef RotaryEncoderAccel_h
#define RotaryEncoderAccel_h

#include "Arduino.h"

#define LATCHSTATE 3

class RotaryEncoderAccel
{
public:
enum class Direction {
    NOROTATION = 0,
    CLOCKWISE = 1,
    COUNTERCLOCKWISE = -1
  };
  
  // ----- Constructor -----
  RotaryEncoderAccel(int pin1, int pin2);

  void calculateDirection();

  // retrieve the current position
  int  getPosition();

   // simple retrieve of the direction the knob was rotated last time. 0 = No rotation, 1 = Clockwise, -1 = Counter Clockwise
  RotaryEncoderAccel::Direction getDirection();

  // adjust the current position
  void setPosition(int newPosition);

  // call this function every some milliseconds or by using an interrupt for handling state changes of the rotary encoder.
  void tick(void);

    // Returns the time in milliseconds between the current observed
  unsigned long getMillisBetweenRotations() const;

  // Returns the RPM
  float getRPM();

  // Switches acceleration mode (fast response)
  void setAccel(unsigned int value1, int value2);

private:
  int _pin1, _pin2; // Arduino pins used for the encoder.

  int8_t _oldState;

  unsigned int accel;
  int multipleter;
  unsigned long prevTick;

  volatile long _position;        // Internal position (4 times _positionExt)
  volatile long _positionExt;     // External position
  volatile long _positionExtPrev; // External position (used only for direction checking)

  unsigned long _positionExtTime;     // The time the last position change was detected.
  unsigned long _positionExtTimePrev; // The time the previous position change was detected.

  bool detectRotation; // If true, the position is only changed if the encoder was rotated.
  Direction _direction; // The direction of the last rotation.
};

#endif

// End
