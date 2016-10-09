# PID

This respository is intended for teaching the general public about PID controllers and gain tuning methods.

To better understand PID, we build a simple mechanical system using a stick mounted to the output shaft of a motor. This stick serves as a disurbance to our motor during operation.

By using an encoder, motor driver, and microcontroller, such as the Arduino Uno, we are able measure the position and velocity of the motors output. From there, we implement a PID controller to stabilze the position and/or velocity of the output shaft.

![PID Setup](/Documents/PID_Setup.jpg?raw=true "PID Setup")