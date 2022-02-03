#  Pitch
## Game Robot - 3rd Year Project
*Created by AMADEI Julien, BERNARD Lucas and GUIGON Louis, using [Alphabot 2](https://www.waveshare.com/w/upload/1/1f/Alphabot2-user-manual-en.pdf), [Ubuntu 20.04](https://doc.ubuntu-fr.org/focal) and [ROS Noetic](http://wiki.ros.org/noetic).*

### Media
* Additional information (report and presentation) are present in the ".media" folder of this repo.
* A video showcasing the robot can also be found [here](https://youtu.be/ZrG38tN--KY).

### Concept
#### Setup
*Users/participants sit around a table.*  
* Pitch is be put on said table, which allows it to stand at the correct height relative to the participants.  
* Pitch should be able to figure out where a human is sitting ***(Pan/Tilt Camera)*** and move ***(CC Motors)*** closer to it

#### Query
Once the necessary distance is reached, Pitch will look at the user and prompt them to play with him by lighting up in green ***(LEDs)***.
* The user can accept by doing a thumbs up, which will cause Pitch to be happy ***(Buzzer)*** and begin the game query by lighting up in white and beeping.
* The user can decline by doing a thumbs down, which will cause Pitch to shutdown.
To choose a game, the user can interact with the IR Remote, by pressing either 1, 2 or 3.
The robot will light up accordingly, and begin playing the selected game.

### Games
#### Game 1 - Rock, Paper, Scissors (Purple LEDs)
* Pitch will vibrate ***(Buzzer)*** and blink ***(LEDs)***, to mimic counting down from 3.
* Once the countdown is over, the LEDs will light up a static color, indicating Pitch’s “choice”.
  * **Green** -> Paper
  * **Red**   -> Scissors
  * **Blue**  -> Rock
* In the case of a tie, the game starts again.*

#### Game 2 - Spin the Wheel (Cyan LEDs)
* The user will be asked to spin a tricolor wheel *(colors to be decided)*.
* PWhile the wheel is spinning, Pitch will decide on a *(random)* color.
* The LEDs will light up a static color, indicating Pitch “choice”.

#### Game 3 – Simon Says (Yellow LEDs)
* Pitch will vibrate and indicate a fixed color using its LEDs.
* The player has 15 seconds to show it an object of the stated color, and Pitch will assess the result.  

### Results
*Pitch's results are assessed using image recognition.*
* Should Pitch win, it will blink ***(LEDs)*** and vibrate ***(Buzzer)***, while spinning around/dancing ***(CC Motors)***, then ask for another game.
* Should Pitch lose, it will blink ***(LEDs)*** and vibrate ***(Buzzer)***, while shaking its head, then ask for another game.
* If the user agrees to another game, the game loop continues.
* If the user says no, Pitch will go back to the game selection loop.

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