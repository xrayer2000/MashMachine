RotaryEncoderAccel
=============

Forked in 2015 and modified for personal project

## Changes:

* Added acceleration support with `setAccel(unsigned int timewindow, int multipleter)`
Meaning that the faster the encoder is rotated the greater the step of increment/decrement will be and keeping in mind that the encoder must be able to step by 1 if rotated slowly enough. Threshold time between steps is the given value. For turn off set it to `0, 0`.
Suggested initial values are `setAccel(200, 1)`.


## Credits

Original library by [**mathertel**](https://github.com/mathertel)

Idea implemented by [**Susensio**](https://github.com/Susensio)

Make it as an independent library by [**lazlyhu**](https://github.com/lazlyhu)

## Original readme

>A Library for the Arduino environment for using a rotary encoder as an input.
>
>Here you can find an Arduino compatible library for using rotary encoders.
>
>I was searching a library for using a rotary encoder in my latest project and found a lot of information on this topic but none of the existing libraries did immediately match my expectations so I finally built my own.
>
>The article on my web site explains the software mechanisms used in detail so you can understand
>the coding and might be able to adjust it to your needs if you like:
>
>http://www.mathertel.de/Arduino/RotaryEncoderLibrary.aspx
>
>
>There are various aspects when writing a library for rotary encoders and you can also find a lot of the sources I analyzed at the bottom of this article.
