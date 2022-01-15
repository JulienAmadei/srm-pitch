#! /usr/bin/env python3

#####################################################################
# Mix of code, pseudo code and code architecture for clearer ideas. #
#####################################################################

# Packages importation
from __future__ import print_function
from re import S
import sys
from tkinter import E
import rospy
import time
#% Add more as necessary !

player_read = False
############################################################################# 
#% Initialisation
def init():
    #% Global init stuff goes here, such as  PINS / global variables / (?)
    #% Functions too
    print("I'm starting up !")
   

############################################################################# 
# User lookup
def look_for_user():
    #% Camera stuff / loop goes here.
        #% To begin, Analyse image at camera starting position
            #% If there is an user, move toward them.
            print("I found someone ! I'll move toward them.")
            #% Movement loop with camera feedback.
                #% Once the position is good (close enough), blink LEDs as signal
                #% Then break the loop.
        #% Look around to find someone (camera pan/tilt, maybe self-rotation if needed)
        print("Seems no one is nearby... I'll look around.")

############################################################################# 
# Query for playing
def wanna_play():
    print("Wanna play ?")
    #% Camera stuff goes here.
        #% Analyse thumbs up or down
            #% If thumbs up, the user "accepts", and Pitch buzzes.
                print("Ok !")
                #% Ask for the game using remote.
            #% If thumbs down, the user "declines", and Pitch looks down and backs away.
            print("Well, maybe another time...")

############################################################################# 
# Defining game-launching functions
def game_RPS():
    #% Launch game 1's main node
    print("Let's play Rock Paper Scissors !")

def game_Wheel():
    #% Launch game 2's main node
    print("Bring out the wheel, please !")

def game_Simon():
    #% Launch game 3's main node
    print("Simon says: do as I do !")

def game_choice(argument): # Switch case - Python version
    print(f"Oh, so you've chosen game number {argument} !")
    game_IDs={
        1:game_RPS,
        2:game_Wheel,
        3:game_Simon
    }
    game = game_IDs.get(argument, lambda: "That's not a game I know... (Invalid game ID)")
    game() # Execute the function corresponding to input

############################################################################# 
if __name__ == '__main__':
    try:
        print('Beginning Pitch Startup')
        init()
        print("I'm ready to go !")
        while(True):
            while(not player_ready):
                look_for_user()
                print('Hello there !')
                player_ready = wanna_play()
                
            while(player_ready): # Forever loop
                #% Get the info from the IR remote (cf. IR client)
                #% (need to change 1/2/3 in game_choice to button IDs !)
                print("What game do you want to play ? Use the remote to tell me !")
                input = readIR
                game_choice(input)
                #% The game ends.
                player_ready = wanna_play()
                print("Okay, thanks for playing !")
    except rospy.ROSInterruptException:
        print("The program (main_init.py) has just been interrupted !", file=sys.stderr)
