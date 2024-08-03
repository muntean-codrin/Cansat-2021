# Cansat-2021

A CanSat is a simulation of a real satellite, integrated within the volume and shape of a soft drink can. The challenge is to fit all the major subsystems found in a satellite, such as power, sensors and a communication system, into this minimal volume. The CanSat is then launched to an altitude of a few hundred metres by a rocket or dropped from a platform or captive balloon and its mission begins: to carry out a scientific experiment and achieve a safe landing.

<img align="left" src="https://herotech.ro/images/cansat/CansatRender.jpg" width="375">
<img src="https://herotech.ro/images/cansat/image4c.png" width="277">

https://user-images.githubusercontent.com/47789965/137624249-1caf85bc-d566-46ed-a9af-3027bbf8992c.mp4


Our CanSats main board controller uses an ATSAMD21G18 chip. We burned a bootloader from Adafruit to it so that it could act as an Adafruit feather m0 board. This was possible because feather m0 uses the same ATSAMD21G18 as a processor. This helped us by opening the possibility to use an Arduino as an IDE to help write, manage and upload C++ code to the board. Another advantage is the already defined configuration for the pins of the board, meaning that we could still use software interfaces such as Serial without having to change the registers or pins configuration.

The CanSat will have 3 phases corresponding to the state of the mission: before launching, launch phase and after landing. To automatically switch between the phases, we are using a state machine system.

Phase 1 – Before Launch. The CanSat will boot up the program at starting. It will calibrate the sensors and make the RGB led green if everything is working properly and the CanSat is ready to start its mission. A blue light is used to indicate a partial initialization, for example if a sensor fails due to different reasons. This is not ideal but the CanSat will be able to complete its mission partly. It is recommended to reboot from the switch if a blue light is showing. A red led is used when a fatal error occurs. This means that the CanSat will not be able to control its descent, or it will not be able to store data collected during it. This phase is working in a low power mode, reading data only from the accelerometer to determine when to switch to the next phase. We are averaging from multiple readings in order to validate the data and not switch to the next phase if the sensor outputs an erroneous value.

Phase 2 – Launch phase. This phase starts when the CanSat detects getting dropped out of the plane/helicopter. At this point the main board will read data from all sensors and will save them to the SD card at a rate of 200 MHz The read data consists of relative time since start, acceleration in the Z axis, pressure, altitude, temperature, longitude, latitude, battery voltage. At the same time, the yaw angle of our CanSat is calculated from the readings of the ICM20948 sensor. The CanSat will always move in the direction of the parachute’s forward vector, so by knowing the yaw angle we also know in which direction the CanSat is heading. We calculate the direction in which the CanSat should be heading by subtracting the landing target coordinates from the current position coordinates and evaluating the arctangent of their division. For turning the CanSat to the desired heading we send commands to the motors in order to retract the ropes. The side where we retract the ropes will act as a pivot for the spinning motion, and the heading will be adjusted, moving the CanSat toward the targeted landing spot.

Phase 3 – After Landing. The CanSat will detect the landing with the help of the accelerometer, using the same concept as the for the launch. After that it will record the GPS position in order to compare it to the targets landing spot and will start the buzzer. It will disable all sensor readings, since the mission has ended and data collection is no longer needed, and since power should be conversed for the battery to last until the CanSat is recovered.

MPL31152A2
This sensor is used to read the air pressure with the help of Adafruit MPL31152A2 library by getting the averages of more sensor’s readings. The output in Pa is used to calculate the altitude in meters with the following formula:

![image](https://github.com/user-attachments/assets/f3cbcafe-1314-461e-8444-fdc394be212d)

TLC59108
We did not find a library for this RGB led that is compatible with Arduino, so we wrote our own by consulting the datasheet. In our program we define an object of type TLC59108 to which we pass the reset pin in the constructor. We defined different methods for this object such as setColor(r, g, b) to which we pass 3 values from 0 to 255, each representing the intensity of the red, green and blue channels.

ICM20948
To read data from this IMU we are using the Sparkfun 9DoF IMU Breakout Arduino Library. By enabling the Digital Motion Processor, the library is able to output the orientation calculated from the fusion of the sensor data from the 3-axis gyroscope, 3-axis accelerometer and 3-axis compass in either Euler or quaternion format. For calculating the compass heading we could technically only use the magnetometer, but the output value would change drastically with a change in roll and pitch of the CanSat. We have to make up for that by plugging those angles in our equations:

![image](https://github.com/user-attachments/assets/fa91295a-8f26-463f-b088-42f7b17b4ee3)

u-Blox Neo-6M
This GPS module provides the global coordinates, UTC time and temperature with the help of the TinyGPS++ library. On a cold start the GPS might take a few minutes to acquire the data broadcasted by the satellites in order to calculate a fix.

Dc Motors
Dc motor control is done via 2 digitals pins per each motor to be able to rotate the motor clockwise or anticlockwise. Each of the 4 quadrature encoders help us determine the absolute rotation of a motor since program start-up by being able to identify rotation counts in both directions. So instead of relying on a feed forward solution in order to actuate the motors, we have a closed loop solution which should be able to generate better results for our descent control system.


