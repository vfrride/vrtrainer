# Description

When learning a new skill there's a lot to balance. Babies are AMAZING they generally figure out a series of coordinated muscle movements in under a year that makes them mobile and able to grasp things. These actions are generally learned by observance, repetition and feedback. The feedback comes in the form of verbal encouragement that they probably can't really understand yet, as well as physical feedback as they learn their balance.

For the TVCOG VR Hackathon my teammates and I decided to create a haptic feedback system to help with task training. We'll be using Google Cardboard as our VR display. Initial system will use an Adafruit Feather bluetooth, 9dof sensors and cellphone vibration motors.

# Details

A more trivial example... I 'play' golf. I use quotes because I'm really bad at it. Despite a lot of improvement from working with a trainer, when I'm not working with them I don't get immediate feedback about my swing. The other problem is that with most skills that require muscle memory external feedback has to be processed through the conscious brain and my brain often gets in the way. I think too much, and react too slowly.

So what I'm really looking for is a way to provide immediate feedback when performing an action in a way that eliminates some of the conscious brain processing that slows down physical learning. The feedback should be visual, but it should also be physical so that I can develop my unconscious reactions more quickly.

## What it does

Simulate an action in VR and provide immediate visual and physical feedback as I perform an action.

## How we built it

In order to provide visual feedback an arm model was developed in Unity for Google Cardboard. The arm is programmed to move through a series of angles to perform an action. Eventually this would be recorded by an expert level skilled person.

To provide the actual positions of the trainee 3 - 9 degree of freedom sensors are mounted on the users arm that can calculate the arm's angles relative to compass directions.

Physical feedback is via small vibration motors mounted to the arms aligned with the axis of the accelerometers. Ideally each axis would have at least 6 motors.

There are 2 communications protocols we had to implement. The first is i2c between the 9dof sensor and an Arduino microprocessor. The second communication protocol was bluetooth low energy which takes the information from the Arduino and sends it to an iPhone 5s mounted running Google Cardboard.

## Challenges we ran into

Both of us were new to most of the technologies we were using although we're both programs. Particularly challenging was configuring the bluetooth module to communicate with the iPhone. Bluetooth is configured via commands that have been available since the first modems were released in the 70's, and the documentation is not very clear.

Another issue was with I2C. Each device on the I2C bus has to have a unique address and the sensors I bought did not have reconfigurable addresses. In order to get over this limitation one can add an intermediary microprocessor that has a programmable I2C address.

## Accomplishments that we're proud of

* We learned a lot!
* Wireless bidirectional communications between the Arduino and iPhone
* Communications via i2c between the sensor and the Arduino
* Created a tiny motor driver for the vibration motors
* Modeled and 3d printed a holder for the Arduino module
* Unity modeling of a multi-axis arm with feedback

# What we learned

Providing immediate feedback via some type of physical feedback really makes VR feel much more realistic. Even just a slight buzz caused a noticeable difference to the experience. This technology is hard to get started with, but overall not that difficult. Except for the bluetooth communications there's a lot of documentation out there to help you get started.

# Hardware

* [Adafruit Feather M0 Bluetooth ME](https://www.adafruit.com/products/2995)
* [Adafruit 9dof sensor](https://www.adafruit.com/products/1714)
