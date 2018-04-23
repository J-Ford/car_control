# car_control
Code relating to the control of a retrofitted Taylor-Dunn Personnel Carrier. 

There are 4 main modules in this reporitory.

- sabertooth_interface: Runs on Arduino and comunicates directly with the motor controllers
- car_control: Runs 2 PID loops that take input encoder and setpoint data and send motor effort commands to the sabertooth interface 
- simulation: An indipendant simulation of the vehicle using ros_control
- us_digital_encoders: Encoder reader for the US digital A2 optical shaft encoders. 
