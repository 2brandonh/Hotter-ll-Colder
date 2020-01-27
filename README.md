# Hotter-ll-Colder
**Most of this code was written by Charles Zloom as specified in the comments of the C file** We adapted the embeddedML functions in order to create an STM32 that could process movement, orientation, and distance to a generated position using machine learning.

# Two State System

Two states: initial calibration

Instruction:

sprintf(msg1, "Before starting Hotter or Colder, you will need to calibrate your device. \n Orient the STM so that you can read the ST symbol left to right. \n Turn the device on its side left, then turn it on its side right. Double tap to start this."

sprintf(msg1, "Now to calibrate turning, KEEP THE DEVICE FLAT and turn the device 90 degrees to the right for the first motion and turn the device 90 degrees to the left for the second motion. \n Double tap to start this.")

sprintf(msg1, "Now to calibrate motion, KEEP THE DEVICE FLAT and push the device forwards for the first motion and push the device backwards for the second motion. \n Double tap to start this.")


states:
1-forward
2-backward
3-turn left
4-turn right
5-check dist
6-East Egg AAAAAHHHHHH too fast give me more time to decide (this does nothing)
