#! /usr/bin/env python3

#####################################################################
# Mix of code, pseudo code and code architecture for clearer ideas. #
#####################################################################

# Packages importation
#% Some are already imported in main_init
from random import randint
from AB2.behaviour.py import colors_array, wanna_play, vibrate, lightup, result_behaviour
#% Add game 2 specific ones.

############################################################################# 
#% Dictionnaries Initialisation
wheel_color_IDs={
    1: "red",
    2: "green",
    3: "blue"
}
wheel_gameSet = False
wheel_playerReady = True
#############################################################################
def wheel_init():
    #% Game 2's own init stuff goes here, such as  PINS / global variables / (?)
    print("Wait a moment ! I'm setting up the Wheel Game playground.")
############################################################################# 
def wheel_pitch_guess():
    #% Generate random choice 
    wheel_guessed_color = wheel_color_IDs[randint(1,3)]
    color_array = colors_array[wheel_guessed_color]
    print(f"I bet it is going to land on... {wheel_guessed_color} ! I'll light up in {wheel_guessed_color} ({color_array})")
    #% Light up LED accordingly
    return wheel_guessed_color

def wheel_camera_analysis():
    #% Camera stuff to find fingers goes here
    print(f"Oh, I see ... {wheel_result_color} !")
    return wheel_result_color

def game_state(wheel_guessed_color,wheel_result_color):
    if wheel_guessed_color == wheel_result_color:
        print("I guessed right !")
        state = True
    else:
        print("Somehow, I was wrong.")
        state = False
    gameSet = True
    return (gameSet,state)

############################################################################# 
if __name__ == '__main__':
    try:
        wheel_init()
        print("I'm ready to go !")
        while(player_ready_wheel):
            while(not gameSet): # Forever loop
                pitch_guess = wheel_pitch_guess()
                wheel_result = wheel_camera_analysis()
                (gameSet,state) = game_state(wheel_result,wheel_result)
            #% Run result.py with state for behavour (happy/sad)
            result_behaviour(state)

            if(state): # Pitch won
                print("Wanna play again ?")
                #% Use the wanna_play function
                wheel_playerReady = wanna_play()
            else:
                print("Let's stop it there.")
                wheel_playerReady = False
                
    except rospy.ROSInterruptException:
        print("The program (main_wheel.py) has just been interrupted !", file=sys.stderr)
