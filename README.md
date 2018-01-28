## Synopsis

`arduino_teleop` is an Arduino sketch that interfaces with one or two Dimension Engineering Sabertooth controllers and a `rosserial` bridge over USB.

## Motivation

The Sabertooth motor controller has a very powerful serial control library, but serial port options are limited when dealing with full Linux-based computers like our Jetson or NUC. Instead, we can use an Arduino as a serial bridge, parsing a message sent through `rosserial` and then passing that along to the Sabertooth motor controllers.

## Installation

This package is formatted as an Arduino sketch for Arduino Uno or compatible boards. By default, the Sabertooth TX pin is set to pin 3, but this can be configured by modifying the source code. Installation is as simple as compiling in Arduino IDE and programming an Uno, then connecting up to a computer over USB and starting a rosserial node.

## Contributors

`arduino_teleop` is authored and maintained by Gregory Meyer, with design input from Kushal Jaligama and Nico Ramirez.
