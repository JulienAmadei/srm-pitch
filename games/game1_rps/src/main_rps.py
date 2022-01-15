#! /usr/bin/env python3

#####################################################################
# Mix of code, pseudo code and code architecture for clearer ideas. #
#####################################################################

# Packages importation
#% Some are already imported in main_init
from random import randint
from AB2.results.py import result_behavior
#% Add game 1 specific ones.

############################################################################# 
#% Initialisation
moves={
    1: "Rock",
    2: "Paper",
    3: "Scissors"
}

colors={
    1: [255, 0, 0], # Red
    2: [0, 255, 0], # Green
    3: [0, 0, 255] # Blue
}

scenarios={ # Win == True | None = Match Nul
    (1,3): True,
    (2,1): True,
    (3,2): True,
    (1,1): None,
    (2,2): None,
    (3,3): None,
    (1,2): False,
    (2,3): False,
    (3,1): False
}
gameSet = False

def init():
    #% Game 1's own init stuff goes here, such as  PINS / global variables / (?)
   
    #% Functions too
    print("Wait a moment ! I'm setting up the RPS playground.")

############################################################################# 
# Defining game-launching functions
def countdown(intTime):
    #% Buzz one second X times to mimic countdown from intTime to 0
    print("3... 2... 1...")
############################################################################# 

def RPS_choice():
    #% Generate random choice 
    pitch_choice = moves[randint(1,3)]
    color = colors[pitch_choice]
    print(f"I'm going with... {pitch_choice} ! I'll light up in {color}")
    #% Light up LED accordingly
    return pitch_choice

def camera_analysis():
    #% Camera stuff to find fingers goes here
    print(f"Oh, so your move is... {user_choice} !")
    return user_choice

def game_state(pitch_choice,user_choice):
    state = scenarios([pitch_choice, user_choice])
    if state == None:
        print("It's a tie ! Let's go again.")
        gameSet = False
    else:
        gameSet = True
    return (gameSet,state)

############################################################################# 
if __name__ == '__main__':
    try:
        init()
        print("I'm ready to go !")

        while(player_ready_RPS):
            while(not gameSet): # Forever loop
                pitch_choice = RPS_choice()
                user_choice = camera_analysis()
                (gameSet,state) = game_state(pitch_choice,user_choice)
            
            #% Run result.py with state for behavour (happy/sad)
            result_behavior(state)
            
            if(state): # Pitch won
                print("Wanna play again ?")
                #% Use the wanna_play function
                player_ready_RPS = wanna_play()
            else:
                print("Let's stop it there.")
                player_ready_RPS = False

    except rospy.ROSInterruptException:
        print("The program (main_rps.py) has just been interrupted !", file=sys.stderr)
