# PID

This respository is intended for teaching the general public about PID controllers and gain tuning methods.

To better understand PID, we build a simple mechanical system using a metal bar mounted to the output shaft of a motor. This bar serves as a disturbance to the motor during operation.

By using a microcontoller, encoder, and motor driver, we are able measure the position and velocity of the motors output shaft. From there, we implement a PID controller to stabilze the position and/or velocity of the output.

![PID Setup](/Documents/PID_Setup.jpg?raw=true "PID Setup")