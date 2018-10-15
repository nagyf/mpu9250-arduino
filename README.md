# Introduction
This project contains a small Arduino program that is used to read data from a [MPU 9250](https://www.invensense.com/download-pdf/mpu-9250-datasheet/) sensor and send it through the serial port.

# Hookup guide

The following example can be used to communicate with the sensor using the [I2C](https://learn.sparkfun.com/tutorials/i2c/all) protocol.

![MPU9250 hookup example](https://raw.githubusercontent.com/nagyf/mpu9250-arduino/master/doc/arduino-and-mpu9250_bb.png)

# Compiling and Running

Compiling this file needs an external library: https://github.com/bolderflight/MPU9250

To install it:

1. Open Arduino IDE
2. Click `Sketch -> Include Library -> Manage Libraries...`
3. Search for `mpu9250`
4. Install the `Bolder Flight Systems MPU9250 by Bryan Taylor`

After this you can compile and upload this sketch as usual.
