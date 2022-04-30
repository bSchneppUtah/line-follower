# line-follower
Project for CS6780/5780

Brian Schnepp
Connor Cousineau

The purpose of this project is to design a system on the STM board/boards that is able to detect and follow any line that it is presented. 

This project is a line follower that detects a change in IR values provided by two IR sensors and two IR emitter and reports to the STM through GPIO input. 

Basic Instruction: 
You will need the following parts:
  2 Encoder motors // we used 2248 encoder count 47:1 ratio motors
  2 STMf0 boards //or similar register writable boards
  2 IR sensor/emitters 
  2 wheels
  2 motor drivers //See schematics
  Some form of chasis to hold the parts and a lot of tape
  
Once you have all that parts you will need to assign pins on the board to be able to control the motors and recieve data from the IR sensor.
Code will need to be written to accept The IR signal and turn on/off the correct motor depending on which sensor is/isn't detecting any input. See the FSM file to get a rough idea of the code flow.

Combining the systems should create a functioning line followr if done properly. 


  
