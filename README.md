# ROBO-Pro-for-ftDuino

The ftDuino Sketch simulates a ROBO TX Controller - allowing to program and control the ftDuino via ROBO Pro (in Online mode). 
The implementation is based on a fork of mr-kubikus' <a href="https://github.com/mr-kubikus/fx1-arduino-parser">fx1-aduino-parser</a> (v0.3).

# How to use it

After starting the sketch on your ftDuino, connect ROBO Pro with your ftDuino. To do so, choose "USB/WLAN/Bluetooth"/"ROBO TX Controller", and than "Bluetooth" (!) and "All COM Interfaces", than select the same COM Interface your Arduino IDE has used.

Now the ftDuino behaves like a ROBO TX Controller; you can control the device via USB from ROBO Pro. To test the connection start the "Interface Test" of ROBO pro.

# Technical Restrictions

Not supported (by technical restrictions):
- TX extensions
- Synchronisation of encoder motors
- I2C commands 
- BT commands 
- fischertechnik ultrasonic distance sensor 

# History

- 02.09.2022: Version 1.0

# License

All contents of this repository are released under Creative Commons Share-alike 3.0.
