# Cansat-2021

A CanSat is a simulation of a real satellite, integrated within the volume and shape of a soft drink can. The challenge is to fit all the major subsystems found in a satellite, such as power, sensors and a communication system, into this minimal volume. The CanSat is then launched to an altitude of a few hundred metres by a rocket or dropped from a platform or captive balloon and its mission begins: to carry out a scientific experiment and achieve a safe landing.

<img align="left" src="https://herotech.ro/images/cansat/CansatRender.jpg" width="375">
<img src="https://herotech.ro/images/cansat/image4c.png" width="277">

https://user-images.githubusercontent.com/47789965/137624249-1caf85bc-d566-46ed-a9af-3027bbf8992c.mp4


## On initialization
- use rgb led to signal if sensors are initialized properly


## Phase 1 (before launch)
- get altitude / acceleration on z axis
- transitions to phase 2 when altitude / acceleration on z axis mets a certain value (must take multiple readings to be sure)
- reads reference gps position before transitioning

## Phase 2 (launch phase)
- reads all sensor data but gps at an interval of 100ms and saves the data to an sd card
- reads gps once in a while (5 seconds?) to determine current position compared to reference position
- actuate the motors to move the parachute in a way that brings the cansat closer to the reference position - need orientation of the cansat for this
- transitions to phase 3 when altitude / acceleration on z axis stagnates

## Phase 3 (recovery phase)
- Registers the landing position
- Enables beep sound


