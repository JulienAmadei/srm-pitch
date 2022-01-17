#! /usr/bin/env python3

#####################################################################
# Mix of code, pseudo code and code architecture for clearer ideas. #
#####################################################################

# Packages importation
#% Some are already imported in main_init
from random import randint
from AB2.behaviour.py import colors_array, wanna_play, vibrate, lightup, result_behaviour
#% Add game 1 specific ones.

############################################################################# 
#% Dictionnaries Initialisation
RPS_moves={
    1: "Rock",
    2: "Paper",
    3: "Scissors"
}
RPS_colors={
    "Rock":[0, 0, 255],
    "Paper": [0, 255, 0],
    "Scissors": [255, 0, 0] 
}
RPS_scenarios={ # Win == True | None = Match Nul
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
RPS_gameSet = False
RPS_playerReady = True
#############################################################################
def RPS_init():
    #% Game 1's own init stuff goes here, such as  PINS / global variables / (?)
    print("Wait a moment ! I'm setting up the RPS playground.")
############################################################################# 
def RPS_pitch_move():
    #% Generate random choice 
    pitch_move = RPS_moves[randint(1,3)]
    indicator_color = colors_array[pitch_move]
    print(f"I'm going with... {pitch_move} ! I'll light up in {RPS_colors}")
    #% Light up LED accordingly with indicator_color
    lightup(indicator_color)
    return pitch_move

def RPS_camera_analysis():
    #% Camera stuff to find fingers goes here
    print(f"Oh, so your move is... {user_move} !")
    return user_move

def RPS_game_state(pitch_move,user_move):
    state = RPS_scenarios([pitch_move, user_move])
    if state == None:
        print("It's a tie ! Let's go again.")
        gameSet = False
    else:
        gameSet = True
    return (gameSet,state)
############################################################################# 
if __name__ == '__main__':
    try:
        RPS_init()
        print("I'm ready to go !")

        while(RPS_playerReady):
            while(not gameSet): # Forever loop
                pitch_move = RPS_pitch_move()
                user_move = RPS_camera_analysis()
                (gameSet,state) = RPS_game_state(pitch_move,user_move)
            
            #% Run results action from behaviour.py for behavour (happy/sad)
            result_behaviour(state)
            
            if(state): # Pitch won
                print("Wanna play again ?")
                #% Use the wanna_play function
                RPS_playerReady = wanna_play()
            else:
                print("Let's stop it there.")
                RPS_playerReady = False

    except rospy.ROSInterruptException:
        print("The program (main_rps.py) has just been interrupted !", file=sys.stderr)
