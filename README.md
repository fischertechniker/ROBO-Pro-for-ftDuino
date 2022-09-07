# ROBO-Pro-for-ftDuino - the "TX Simulator Sketch"

The ftDuino Sketch simulates a ROBO TX Controller - allowing to program and control the ftDuino via ROBO Pro (in Online mode). 
The implementation is based on a fork of mr-kubikus' <a href="https://github.com/mr-kubikus/fx1-arduino-parser">fx1-aduino-parser</a> (v0.3).

# How to use it

After starting the sketch on your ftDuino, connect ROBO Pro with your ftDuino. To do so, choose "USB/WLAN/Bluetooth"/"ROBO TX Controller", and than "Bluetooth" (!) and "All COM Interfaces", than select the same COM Interface your Arduino IDE has used.

Now the ftDuino behaves like a ROBO TX Controller; you can control the device via USB from ROBO Pro. To test the connection start the "Interface Test" of ROBO pro.

# Functionality

Running in online mode, ROBO Pro Programs send request packets to the ROBO TX Controller to configure input and output ports, read inputs, initiate I2C sensors or actors communication and activate motor commands. The "TX Simulator Sketch" is able to execute the following operational elements like a ROBO TX Controller:
- configuration of input ports (analog/voltage, analog/resistance, digital/voltage, digital/resistance)
- read of (fischertechnik) sensor values (depending on the respective configuration), exept for the ultrasonic distance sensor
- configuration, reset and read of counter values
- set PWM values of output ports
- activate motor commands (speed/PWM, direction)
- activate and control extended motor commands (distance)
- I2C read and write commands

# Technical Restrictions

Not supported (by technical restrictions):
- control of TX extensions
- synchronisation of encoder motors
- bluetooth communication 
- use of the fischertechnik ultrasonic distance sensor (alternative: I2C sensor SRF02 or SRF08)

# History

- Version 1.0 (02.09.2022), testet with TX Firmware 1.30 and ROBO Pro 4.6.6
- Version 1.1 (07.09.2022), I2C support added, digital line sensor values

# License

All contents of this repository are released under <a href="https://creativecommons.org/licenses/by-sa/3.0/">Creative Commons Share-alike 3.0</a>.
