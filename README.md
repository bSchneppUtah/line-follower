
# line-follower
## Project for CS6780/5780
____
### Team
 - Brian Schnepp
 - Connor Cousineau

### Summary

This project implements a line follower based around two STM32F0 development boards, two sensor modules, two motors, and some other components. It is capable of detecting any line on a surface with black tape, and following it to it's conclusion.

Information provided by both IR sensor modules can be used and compared to determine what direction and how fast to control the motor. The module provided converts the distance between itself and some distant object into a digital signal which determines if some dark object is present at that distance. This is used with some manual tuning of the sensors to have a consistent range check for the line below the robot.
For both STM32 boards, the output for the sensor modules is read as input in PC0.

### Bill of Materials
| Component  | Count  | Store Link  |
|---|---|---|
|2248 47:1 Encoder Motor|  2 |  ADD ME |
|STM32F0 Development Board|  2 |https://www.digikey.com/en/products/detail/stmicroelectronics/STM32F0DISCOVERY/3045359|
|IR Sensor|  2 |https://www.amazon.com/HiLetgo-Infrared-Avoidance-Reflective-Photoelectric/dp/B07W97H2WS|
|Wheels | 2 | ADD ME |
|Motor Drivers | 2 | See HW4 Solution |
|Chassis | 1 | 3D print or adapt acrylic frame|
|Jumper Wires | 23 | https://www.amazon.com/EDGELEC-Breadboard-Optional-Assorted-Multicolored/dp/B07GD2BWPY
|Dark Tape | 1 Roll | https://www.amazon.com/Gorilla-Tape-Mini-Travel-Black/dp/B01M2AAGTZ

### Wiring Diagram
The wiring diagram to set the board up is as follows:
[Wiring Diagram](WiringDiagram.png)
  
Once you have all that parts you will need to assign pins on the board to be able to control the motors and recieve data from the IR sensor.
Code will need to be written to accept The IR signal and turn on/off the correct motor depending on which sensor is/isn't detecting any input. See the FSM file to get a rough idea of the code flow.

Combining the systems should create a functioning line follower if done properly. 

The zip only contains files related to the motor driver. 
  
