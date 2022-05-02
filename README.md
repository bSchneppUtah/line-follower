
# line-follower
Project for CS6780/5780
____
### Team
 - Brian Schnepp
 - Connor Cousineau

### Summary

This project implements a line follower based around two STM32F0 development boards, two sensor modules, two motors, and some other components. It is capable of detecting any line on a surface with black tape, and following it to its conclusion.

Information provided by both IR sensor modules can be used and compared to determine what direction and how fast to control the motor. The module provided converts the distance between itself and some distant object into a digital signal which determines if some dark object is present at that distance. This is used with some manual tuning of the sensors to have a consistent range check for the line below the robot.
For both STM32 boards, the output for the sensor modules is read as input in PC0.

### Bill of Materials
| Component  | Count  | Store Link  |
|---|---|---|
|2248 47:1 Encoder Motor|  2 |  ADD ME |
|STM32F0 Development Board|  2 |https://www.digikey.com/en/products/detail/stmicroelectronics/STM32F0DISCOVERY/3045359|
|IR Sensor|  2 |https://www.amazon.com/HiLetgo-Infrared-Avoidance-Reflective-Photoelectric/dp/B07W97H2WS|
|Wheels | 2 | https://www.amazon.com/Smart-Chassis-Motors-Encoder-Battery/dp/B01LXY7CM3/ref=sr_1_3?keywords=emo+smart+robot+car+chassis+kit&qid=1651531657&sprefix=EMO+smart+robot+%2Caps%2C129&sr=8-3 |
|Motor Drivers | 2 | See HW4 Solution |
|Chassis | 1 | 3D print or adapt acrylic frame & https://www.amazon.com/Smart-Chassis-Motors-Encoder-Battery/dp/B01LXY7CM3/ref=sr_1_3?keywords=emo+smart+robot+car+chassis+kit&qid=1651531657&sprefix=EMO+smart+robot+%2Caps%2C129&sr=8-3|
|Jumper Wires | 23 | https://www.amazon.com/EDGELEC-Breadboard-Optional-Assorted-Multicolored/dp/B07GD2BWPY
|Dark-Color Tape | 1 Roll | https://www.amazon.com/Gorilla-Tape-Mini-Travel-Black/dp/B01M2AAGTZ
| 12V AC Wall Adapter | 2 | https://www.amazon.com/TMEZON-Power-Adapter-Supply-2-1mm/dp/B00Q2E5IXW/
| USB Mini B Cable | 2 | https://www.amazon.com/Monoprice-15-Feet-Mini-B-Ferrite-105450/dp/B002KL8N6A/

### Wiring Diagram
The wiring diagram to set the board up is as follows:
![Wiring Diagram](WiringDiragram.png)

### Detailed Setup and Usage
Once all required materials have been gathered, the system can then be set up.  Some tape may be necessary for additional structural security of certain components to the final board, e.g., a motor driver. The IR sensors should be secured to the front of the vehicle, with both sensors facing downwards toward the area where a line is expected to be seen. Both boards should be flashed with appropriate firmware for their placement: A macro ```right``` should only be defined in ```main.c``` for the right side board. Lastly, both wheels should be attached to a motor, and the motor then appropriately mounted to the chassis. It is then important to consult the wiring diagram for properly connecting each component together.

Once the project has been set up, the robot should be ready for usage. Draw some pattern on the ground, being careful to avoid excessively small or sharp distances smaller than about the diameter of one of the wheels, where the robot should follow. When both are sensing, the robot should continue straight. When only one sensor is detecting, the side with the sensor not detecting should have it's motor disabled. When both do not sense, then the robot should stop moving. Please consult the finite state machine diagram for a more detailed view:
![FSM Model](58F1D1B7-348A-478D-A0A1-7C6BF3568DC1.jpg)
The zip only contains files related to the motor driver. 
  
## Development Progress
TODO
