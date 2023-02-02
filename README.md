# Lofty-2023
The in-season code for Lofty (Working name), our robot for the FRC Charged Up 2023 Season. 


## Functions
Functional Swerve drive code

## Controls
### Driver (USB 0)
#### Left Stick (X and Y axes)
Chassis Strafe
#### Right Stick (X axis)
Chassis rotate
#### A button
toggle robot/field oriented
#### B button
reset yaw to 0


## NT Docs /
### general/
#### PDH/
The associated REV PDH (CAN ID 1)
##### voltage
The input voltage from the battery
##### current
The total current draw of the robot on the battery
##### temperature
The temperature of the PDH
### drivetrain/
#### isRobotOriented
#### gyro/
##### yaw
##### pitch
##### roll
#### PathPlanner/
##### translationKp
##### translationKi
##### translationKd
##### rotationKp
##### rotationKi
##### rotationKd
#### modules/
##### FL/
###### drive/
targetSpeed<br>
actualSpeed<br>
temperature<br>
outputVoltage<br>
statorCurrent<br>
###### azimuth/
targetPosition<br>
actualPosition<br>
temperature<br>
outputVoltage<br>
statorCurrent<br>
##### FR/
###### drive/
targetSpeed<br>
actualSpeed<br>
temperature<br>
outputVoltage<br>
statorCurrent<br>
###### azimuth/
targetPosition<br>
actualPosition<br>
temperature<br>
outputVoltage<br>
statorCurrent<br>
##### BL/
###### drive/
targetSpeed<br>
actualSpeed<br>
temperature<br>
outputVoltage<br>
statorCurrent<br>
###### azimuth/
targetPosition<br>
actualPosition<br>
temperature<br>
outputVoltage<br>
statorCurrent<br>
##### BR/
###### drive/
targetSpeed<br>
actualSpeed<br>
temperature<br>
outputVoltage<br>
statorCurrent<br>
###### azimuth/
targetPosition<br>
actualPosition<br>
temperature<br>
outputVoltage<br>
statorCurrent<br>
#### kinematics/
##### clockwiseSpeed
##### robot/
###### forwardSpeed
###### rightwardSpeed
##### field/
###### DSawaySpeed
###### DSrightSpeed
#### odometry/
##### field/
###### DSawayPosition
###### DSrightPosition
