#! /usr/bin/env python3

#####################################################################
# Mix of code, pseudo code and code architecture for clearer ideas. #
#####################################################################

# Packages importation
#% Some are already imported in main_init
from random import randint
#% Add game 1 specific ones.

############################################################################# 
#% Initialisation
colors_IDs={
    1: "red",
    2: "green",
    3: "blue"
}

colors_array={
    "red": [255, 0, 0],
    "green": [0, 255, 0], # Green
    "blue": [0, 0, 255]  # Blue
}

gameSet = False

def init():
    #% Game 2's own init stuff goes here, such as  PINS / global variables / (?)
   
    #% Functions too
    print("Wait a moment ! I'm setting up the Wheel Game playground.")

############################################################################# 
# Defining game-launching functions
def countdown(intTime):
    #% Buzz one second X times to mimic countdown from intTime to 0
    print("3... 2... 1...")
############################################################################# 

def wheel_choice():
    #% Generate random choice 
    color_choice = colors_IDs[randint(1,3)]
    color_array = colors_array[color_choice]
    print(f"I bet it is going to land on... {color_choice} ! I'll light up in {color_choice} ({color_array})")
    #% Light up LED accordingly
    return color_array

def camera_analysis():
    #% Camera stuff to find fingers goes here
    print(f"Oh, I see ... {seen_color} !")
    return seen_color

def game_state(pitch_choice,user_choice):
    if color_array == seen_color:
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
        init()
        print("I'm ready to go !")
        while(not gameSet): # Forever loop
            pitch_choice = wheel_choice()
            user_result = camera_analysis()
            (gameSet,state) = game_state(pitch_choice,user_result)
        #% Run result.py with state for behavour (happy/sad)
        pitch_result(state)

        if(state): # Pitch won
            print("Wanna play again ?")
            #% Use the wanna_play function
            player_ready_wheel = wanna_play()
        else:
            print("Let's stop it there.")
            player_ready_wheel = False
                
    except rospy.ROSInterruptException:
        print("The program (main_wheel.py) has just been interrupted !", file=sys.stderr)
