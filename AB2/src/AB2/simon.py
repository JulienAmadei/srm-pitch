#! /usr/bin/env python3

#####################################################################
# Mix of code, pseudo code and code architecture for clearer ideas. #
#####################################################################

# Packages importation
#% Some are already imported in main_init
from random import randint
from AB2.behaviour.py import colors_array, wanna_play, vibrate, lightup, result_behaviour
#% Add game 3 specific ones.

############################################################################# 
#% Dictionnaries Initialisation
simon_colors_IDs={
    1: "red",
    2: "green",
    3: "blue"
}
simon_colors_array={
    "red": [255, 0, 0],
    "green": [0, 255, 0], 
    "blue": [0, 0, 255] 
}
simon_gameSet = False
############################################################################# 
def simon_init():
    #% Game 3's own init stuff goes here, such as  PINS / global variables / (?)
    print("Wait a moment ! I'm setting up the Simon Says Game playground.")
############################################################################# 

def simon_choice():
    #% Generate random choice 
    color_choice = simon_colors_IDs[randint(1,3)]
    color_array = simon_colors_array[color_choice]
    print(f"I want you to show me something that is {color_choice} ! I'll light up in {color_choice} ({color_array}) to tell you.")
    #% Light up LED accordingly
    return color_array

def simon_camera_analysis():
    #% Camera stuff to find fingers goes here
    print(f"Oh, I see ... {seen_color} !")
    return seen_color

def simon_game_state(simon_choice,seen_color):
    if simon_choice == seen_color:
        print("Nicely done !")
        state = True
    else:
        print(f"I believe that is {seen_color}, but I asked for {simon_choice}...")
        state = False
    gameSet = True
    return (gameSet,state)

############################################################################# 
if __name__ == '__main__':
    try:
        simon_init()
        print("I'm ready to go !")
        pitch_choice = simon_choice()
        while(simon_playerReady):
            while(not gameSet): # Forever loop
                user_result = simon_camera_analysis()
                (gameSet,state) = simon_game_state(pitch_choice,user_result)
            #% Run result.py with state for behavour (happy/sad)
            result_behaviour(state)

            if(state): # Pitch won
                print("Wanna play again ?")
                #% Use the wanna_play function
                simon_playerReady = wanna_play()
            else:
                print("Let's stop it there.")
                simon_playerReady = False
                
    except rospy.ROSInterruptException:
        print("The program (main_simon.py) has just been interrupted !", file=sys.stderr)
