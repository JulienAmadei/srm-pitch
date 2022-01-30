#  Pitch
## Game Robot - 3rd Year Project
*Created by AMADEI Julien, BERNARD Lucas and GUIGON Louis, using [Alphabot 2](https://www.waveshare.com/w/upload/1/1f/Alphabot2-user-manual-en.pdf) and ROS.*

### Concept
#### Setup
*Users/participants sit around a table.*  
* Pitch is be put on said table, which allows it to stand at the correct height relative to the participants.  
* Pitch should be able to figure out where a human is sitting ***(Camera)*** and move ***(CC Motors)*** closer to it
***(Proximity Sensors – IR & Ultrasound OR IR Line Sensors)***.  
* The process begins by activating the game ***(Remote)***.

#### Query
Once the necessary distance is reached, Pitch will look at the user and prompt them to play with him by blinking several times ***(LEDs)***.
* The user can accept by doing a thumbs up, which will cause Pitch to be happy ***(Buzzer)*** and begin playing.
* The user can decline by doing a thumbs down, which will cause Pitch to look down sadly and to back away.

### Games
#### Game 1 - Rock, Paper, Scissors
* Pitch will vibrate ***(Buzzer)*** and blink ***(LEDs)***, to mimic counting down from 3.
* Once the countdown is over, the LEDs will light up a static color, indicating Pitch’s “choice”.
  * **Green** -> Paper
  * **Red**   -> Scissors
  * **Blue**  -> Rock

#### Game 2 - Spin the Wheel
* The user will be asked to spin a tricolor wheel *(colors to be decided)*.
* Pitch will vibrate ***(Buzzer)*** and blink ***(LEDs)***, while the wheel is spinning, but will decide a *(random)* color.
* Once the wheel stops spinning is achieved, the LEDs will light up a static color, indicating Pitch “choice”.

#### Game 3 – Simon Says
* Pitch will vibrate and indicate a fixed color using its LEDs.
* The player must show him an object of the stated color, and Pitch will assess the result.  
*(To admit defeat, the user must cover the camera until Pitch vibrates and the LEDs stop lighting up.)*

### Results
*Pitch's results are assessed using image recognition.*
* Should Pitch win, it will blink ***(LEDs)*** and vibrate ***(Buzzer)***, while spinning around/dancing ***(CC Motors)***, then ask for another game.
* If the user agrees to another game, the game loop continues.
* If the user says no, Pitch will act as if the game was declined.

### Hardware
* 2 IR Sensors for Object-proximity sensing
* 1 Ultrasound-Proximity Sensor – Not necessarily present, but there are pins for a HC-SR04 type sensor.
* 2 CC Motors
* 1 Color Camera (Logitech C270 - Pannable & Tiltable)
* 1 Buzzer
* 1 Joystick (unused in our use case)
* 4 RGB LEDs
* 1 IR Remote
* 5 IR Sensors for Line Tracking