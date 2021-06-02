# autonomous-drone-
Autonomous control of a crazyflie 2.X quadcopter.
To execute this program it is mandatory to have downloaded the cflib provided by bitcraze.

# Functionalities

This program allows a crazyflie to take off from a 0.3x0.3x0.1 [m] pad, with location previousy known, and go, while avoiding obstacles, to a search zone where a simular landing pad is located. Once in this zone, the quadcopter search for the box and land on it. After it, it goes back to the inital take off pad.
