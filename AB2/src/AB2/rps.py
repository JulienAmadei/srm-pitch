#! /usr/bin/env python3

#####################################################################
# Mix of code, pseudo code and code architecture for clearer ideas. #
#####################################################################

# Packages importation
from __future__ import print_function
from re import S
from random import randint
import sys
from tkinter import E
import rospy
import roslib
import time
from std_msgs.msg import String 

# Services/Actions Importation and functions
from camera.srv import SwitchService
from led.srv import *
from buzzer.srv import *
import actionlib
import motor.msg
import servo.msg

############################################################################# 
def game_logic_RPS():
    try:
        RPS_init()
        print("I'm ready to go !")

        while(RPS_playerReady == 1):
            while(not gameSet): # Forever loop
                pitch_move = RPS_pitch_move()
                user_move = RPS_camera_analysis()
                (gameSet,state) = RPS_game_state(pitch_move,user_move)
            
            if(state): # Pitch won
                led_blink_client(6,1,[255,0,255])
                motor_client('lft')
                buzzer_client(0.5)
                motor_client('rgt')
                time.sleep(1)
                buzzer_client(0.5)
                motor_client('fwd')
                time.sleep(1)
                buzzer_client(0.5)
                motor_client('bwd')
                time.sleep(1)
                buzzer_client(0.5)
                time.sleep(1)                                
                print("Wanna play again ?")
                #% Use the wanna_play function
                RPS_playerReady = wanna_play()
            else:
                led_blink_client(0,2,[255,0,0])
                buzzer_client(2)
                led_blink_client(0,2,[0,0,0])                
                print("Let's stop it there.")
                RPS_playerReady = False

    except rospy.ROSInterruptException:
        print("The program (main_rps.py) has just been interrupted !", file=sys.stderr)


def RPS_game_state(pitch_move,user_move):
    state = RPS_scenarios([pitch_move, user_move])
    if state == None:
        print("It's a tie ! Let's go again.")
        gameSet = False
    else:
        gameSet = True
    return (gameSet,state)
############################################################################# 
def RPS_pitch_move():
    #% Generate random choice 
    pitch_move = RPS_moves[randint(1,3)]
    indicator_color = RPS_colors[pitch_move]
    print(f"I'm going with... {pitch_move} ! I'll light up in {indicator_color}")
    led_blink_client(0,2,indicator_color)
    return pitch_move

def RPS_camera_analysis():
    t = 0
    #% Camera stuff / loop goes here.
    rospy.wait_for_service('camera_service')
    while(t<3): # 3 Tests
        print(t)
        obj_to_detect = 'hand'
        var1 = camera_client(obj_to_detect)
        nb_finger = var1[0]
        thumb_state = var1[1]
        t+=1
        print(nb_finger)
        if(nb_finger >= 4):
            user_move = 2        
        if(nb_finger < 2):
            user_move = 3     
        else:
            user_move = 1 
    print(f"Oh, so your move is... {user_move} !")
    return user_move
#############################################################################
def RPS_init():
    #% Game 1's own init stuff goes here, such as  PINS / global variables / (?)
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
    global RPS_gameSet
    RPS_gameSet = False
    global RPS_playerReady
    RPS_playerReady = 1
    print("Wait a moment ! I'm setting up the RPS playground.")
