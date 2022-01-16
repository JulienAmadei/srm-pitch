# FICHIER CLIENT MULTIPLE
# RANGE LES COMPORTEMENTS (Séries de mouvements, réactions, etc...)
# Dans un seul fichier pour accès facile

import rospy
# Color for lightup
global colors_array

colors_array={
    1: [255, 0, 0], # Red
    2: [0, 255, 0], # Green
    3: [0, 0, 255] # Blue
}
#############################################################################
# Function used as a query to ask an user for playing
def wanna_play():
    print("Wanna play ?")
    #% Camera stuff goes here.
        #% Analyse thumbs up or down
            #% If thumbs up, the user "accepts", and Pitch buzzes.
                print("Ok !")
                #% Ask for the game using remote.
            #% If thumbs down, the user "declines", and Pitch looks down and backs away.
            print("Well, maybe another time...")
    return player_ready
#############################################################################
# Do a "Countdown" by buzzing
def vibrate(intTime):
    #% Use buzzer one second X times to mimic countdown from intTime to 0
    print("3... 2... 1...")
#############################################################################
# Light up leds
def lightup(color):
    #% it's all in the title
#############################################################################
# Moves pitch and does a serie of actions. state = True is a win, False is a loss
def result_behaviour(state):
    #% it's all in the title
    if state == True # Win
    # If the robot won:
    # It blinks (LEDs)
    # It vibrates (Buzzer)
    # Spins arround while spinning (CC Motors)
    else # Loss
    # If the robot lost, it vibrates out of rage