Author : Salim KHAZEM 

License : GPL 3  


RC Rover is a robotics project that aims to improve rover control through the use of radio frequency and the interaction of rover movement with hand movement using the inertial unit (MPU6050), but also intends to control of this rover by using a joystick. All this is done remotely using the radio frequency Nrf24l01 (2.4Ghz).

This project is realized using open source development boards (Arduino): one for data transmitter (main command) which contains the joystick and the inertial unit, and one for the receiver (control of the engines). For the transmission I used Arduino Pro Mini Board and for the receiver I used Arduino Uno board.

I used the joystick and inertial unit (MPU6050) for the command, and the commande is with the RF (radio frequency) using the module Nrf24l01.
