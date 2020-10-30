# Hotter-ll-Colder
**Most of this code was written by Charles Zloom as specified in the comments of the C file.** (ACTUALLY-THE-FINAL-MAIN.c) We adapted the embeddedML functions in order to create an STM32 that could process movement, orientation, and distance to a generated position using machine learning. <br/>

The video detailing a split screen of our device movements and the corresponding code in terminal can be viewed below.
https://www.youtube.com/watch?v=wM6sbqFG2L0&t=120s&ab_channel=BlakeLazarine

<img src=https://github.com/2brandonh/Hotter-ll-Colder/blob/master/HLC1.png width=600>

# Two State System
The device works by first giving the user instructions on how to train the sensortile to process movements according to direction. The objective of the game involves finding a randomized location in a grid by moving around in realtime. At the start of each round, the user can input six different actions based on the two state recognition system as outlined below. <br/> <br/>
<img src=https://github.com/2brandonh/Hotter-ll-Colder/blob/master/HLC2.png width=600> <br/>

Information is given as to the current direction of the sensortile, X and Y units, and temperature in relation to the distance from the final destination. The sensortile blinks in response to the distance as well, and blinks rapidly once the destination is reached. This allows for the user to play the game remotely without requiring the terminal screen.<br/> <br/>
<img src=https://github.com/2brandonh/Hotter-ll-Colder/blob/master/HLC3.png width=600> <br/>
