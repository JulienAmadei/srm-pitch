#! /usr/bin/env python3

#####################################################################
# Importation of the necessary functions/packages/services/actions. #
#####################################################################
# Packages
from __future__ import print_function
from re import S
from random import randint
import sys
from tkinter import E
import rospy
import roslib
import time
from std_msgs.msg import String

# Services
from camera.srv import SwitchService
from led.srv import *
from buzzer.srv import *

# Actions
import actionlib
import motor.msg
import servo.msg

#####################################################################
#  Clients/Listeners for the various services, talkers and actions. #
#####################################################################
def camera_client(data):
    try:
        switch = rospy.ServiceProxy("camera_service", SwitchService)
        res = switch(data)
        return res.nb_finger, res.thumb_state, res.distance
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def motor_client(direction):
    client = actionlib.SimpleActionClient("motor_action_server", motor.msg.MotorAction)
    client.wait_for_server()
    goal = motor.msg.MotorGoal(order=direction)
    client.send_goal(goal)
    time.sleep(1)
    client.cancel_goal()
    client.wait_for_result()
    return client.get_result()


def servo_client(H, V):
    client = actionlib.SimpleActionClient("servo_action_server", servo.msg.ServoAction)
    print("Client created !")
    client.wait_for_server()
    goal = servo.msg.ServoGoal(H, V)
    client.send_goal(goal)
    print(f"Goal sent (H:{goal.H}, V:{goal.V})")


def led_blink_client(time, mode, color):
    rospy.wait_for_service("led_blink")
    try:
        blink = rospy.ServiceProxy("led_blink", BlinkLED)
        blink.call(BlinkLEDRequest(time, mode, color))
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def buzzer_client(requestedTime):
    rospy.wait_for_service("buzzer")
    try:
        buzzerProxy = rospy.ServiceProxy("buzzer", Buzzer)
        resp = buzzerProxy.call(BuzzerRequest(requestedTime))

        if not resp.buzzState == True:
            raise Exception(
                "End of Call failure, returned resp was %f" % resp.buzzerState
            )
        return resp.buzzState

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def ir_callback(data):
    global chosen_game_ID
    chosen_game_ID = 0
    msg = data.data
    print(f"msg : {msg}")
    if msg == "1 (0x0C)":
        chosen_game_ID = 1
    if msg == "2 (0x18)":
        chosen_game_ID = 2
    if msg == "3 (0x5E)":
        chosen_game_ID = 3

##############################################################
#  Pitch-specific functions (intialisation, camera, etc...). #
##############################################################


def pitch_init():
    print("I'm starting up !")
    rospy.init_node("pitch_client")
    rospy.Subscriber("remote_input", String, ir_callback)
    servo_client(0, -30)
    led_blink_client(0, 2, [0, 0, 0])
    # Global Pitch Variables
    global user_distance
    user_distance = -1
    global min_treshold_distance
    min_treshold_distance = 40
    global max_treshold_distance
    max_treshold_distance = 100
    global thumb_state
    thumb_state = 0
    global nb_finger
    nb_finger = -1
    global chosen_game_ID
    chosen_game_ID = 0
    global game_begin
    game_begin = False


def look_for_user():
    t = 0
    rospy.wait_for_service("camera_service")
    while t < 5:  # 5 Tests before validation
        print(t)
        obj_to_detect = "face"
        detectedvar = camera_client(obj_to_detect)
        measured_distance = detectedvar[2]
        t = 1
        print(measured_distance)
        if measured_distance != -1:
            if measured_distance > max_treshold_distance:
                print("You're a bit far ! I'm coming closer.")
                motor_client("fwd")
            if measured_distance < min_treshold_distance:
                print("Too close ! Let me back away a little bit.")
                motor_client("bwd")
        else:
            print("I can't find anyone...")
    return measured_distance


def wanna_play():
    t = 0
    rospy.wait_for_service("camera_service")
    while t < 3:  # 3 Tests before validation
        print(t)
        obj_to_detect = "hand"
        var1 = camera_client(obj_to_detect)
        nb_finger = var1[0]
        thumb_state = var1[1]
        t = 1
        print(thumb_state)
        if nb_finger != -1 and thumb_state != 0:
            if thumb_state == 1:
                print("You said yes !")
            if thumb_state == -1:
                print("You said no..")
        else:
            print("I can't see your fingers...")
    return thumb_state


######################################################
#  Game 1-specific functions (Rock, Paper, Scissors) #
######################################################
def RPS_init():
    global RPS_moves
    RPS_moves = {1: "Rock", 2: "Paper", 3: "Scissors"}
    global RPS_colors
    RPS_colors = {"Rock": [0, 0, 255], "Paper": [0, 255, 0], "Scissors": [255, 0, 0]}
    global RPS_scenarios
    RPS_scenarios = {  # Win == True | Tie == None
        (1, 3): True,
        (2, 1): True,
        (3, 2): True,
        (1, 1): None,
        (2, 2): None,
        (3, 3): None,
        (1, 2): False,
        (2, 3): False,
        (3, 1): False,
    }
    global RPS_gameSet
    RPS_gameSet = False
    global RPS_playerReady
    RPS_playerReady = 1
    print("Wait a moment ! I'm setting up the RPS playground.")


def RPS_pitch_move():
    pitch_move = randint(1, 3)
    indicator_color = RPS_colors[RPS_moves[pitch_move]]
    print(
        f"I'm going with... {RPS_moves[pitch_move]} ! I'll light up in {indicator_color}"
    )
    led_blink_client(0, 2, indicator_color)
    return pitch_move


def RPS_camera_analysis():
    t = 0
    rospy.wait_for_service("camera_service")
    while t < 3:  # 3 Tests
        print(t)
        obj_to_detect = "hand"
        var1 = camera_client(obj_to_detect)
        nb_finger = var1[0]
        thumb_state = var1[1]
        t = 1
        print(nb_finger)
        if nb_finger >= 4:
            user_move = 2
        if nb_finger < 2:
            user_move = 3
        else:
            user_move = 1
    print(f"Oh, so your move is... {RPS_moves[user_move]} !")
    return user_move


def RPS_game_state(pitch_move, user_move):
    state = RPS_scenarios[(pitch_move, user_move)]
    if state == None:
        print("It's a tie ! Let's go again.")
        gameSet = False
    else:
        gameSet = True
    return (gameSet, state)


#######################################################
# MAIN ROBOT CALLS, LOGIC AND LOOPS (execute as main) #
#######################################################

if __name__ == "__main__":
    try:
        print("Beginning Pitch Startup")
        pitch_init()
        print("Ready to go !")
        while True: # Infinite Loop, after initialisation
            while ( # Loop while the user is not within range (tresholds) or not found (-1)
                user_distance == -1
                or user_distance > max_treshold_distance
                or user_distance < min_treshold_distance
            ):
                user_distance = look_for_user() # Register the current user distance
                if user_distance == -1: # If no user is found nearby,
                    servo_client(randint(-50, 50), -30) # randomly move servos
            print("Found someone !")
            led_blink_client(0, 2, [0, 255, 0]) # Blink Green to have visual feedback
            buzzer_client(0.2) # Buzz for sound feedback
            led_blink_client(0, 2, [0, 0, 0]) # Turn off the LEDs
            player_ready = wanna_play() # Ask the found user if they want to play
            while nb_finger != 1 and thumb_state == 0: # As long as the thumb state (up/down) is unsure, and the number of fingers not found
                thumb_state = wanna_play() # Loop
            if thumb_state == 1: # If the user wants to play (thumbs up)
                led_blink_client(0, 2, [255, 255, 255]) # Blink White to have visual feedback
                buzzer_client(0.2)  # Buzz for sound feedback
                print("What game do you want to play ? Use the IR remote to tell me !")
                while True:  # Forever loop for query
                    if chosen_game_ID == 1: # ID 1 - RPS
                        led_blink_client(0, 2, [255, 0, 255]) # Blink Purple to show selection
                        RPS_init() # Initialize the RPS playground.
                        chosen_game_ID = 0 # Cancel chosen_game_ID, as unused as of now
                        while RPS_playerReady == 1: # Player is considered ready as they enter the game
                            buzzer_client(0.2)  # This
                            time.sleep(0.5)     # is
                            buzzer_client(0.2)  # a
                            time.sleep(0.5)     # small
                            buzzer_client(0.5)  # countdown
                            while not RPS_gameSet:  # Loop on ties
                                user_move = RPS_camera_analysis() # See what the user chose
                                pitch_move = RPS_pitch_move()     # Generate a random choice for the robot
                                (RPS_gameSet, state) = RPS_game_state(pitch_move, user_move) # Use game logic to see who won
                            if state:  # Pitch won
                                RPS_playerReady = 0
                                led_blink_client(6, 1, [255, 0, 255]) # Blink for visual feedback
                                motor_client("lft") # Pitch
                                buzzer_client(0.5)  # is
                                motor_client("rgt") # now
                                time.sleep(1)       # dancing
                                buzzer_client(0.5)  # after 
                                motor_client("fwd") # its
                                time.sleep(1)       # win
                                buzzer_client(0.5)  # for
                                motor_client("bwd") # about
                                time.sleep(1)       # six
                                buzzer_client(0.5)  # seconds
                                time.sleep(1)       # roughly
                                print("Wanna play again ?")
                                #% Use the wanna_play function
                                while RPS_playerReady == 0: # Wait for user feedback
                                    RPS_playerReady = wanna_play()
                            else: # Pitch lost
                                led_blink_client(0, 2, [255, 0, 0]) # Blink red for visual feedback
                                buzzer_client(2) # Buzz out of rage
                                led_blink_client(0, 2, [0, 0, 0]) # Turn off leds
                                print("Let's stop it there.") # Sore loser
                                RPS_playerReady = -1
                            break
                    if chosen_game_ID == 2: # ID 2 - Wheel
                        led_blink_client(0, 2, [0, 255, 255]) # Blink Turquoise to show selection
                        led_blink_client(2, 1, [0, 0, 0]) # Turn off LEDs
                        buzzer_client(0.2)  # This
                        time.sleep(0.5)     # is
                        buzzer_client(0.2)  # yet
                        time.sleep(0.5)     # another
                        buzzer_client(0.5)  # cooldown

                    if chosen_game_ID == 3: # ID 3 - Wheel
                        led_blink_client(0, 2, [255, 255, 0]) # Blink Yellow to show selection
                        led_blink_client(2, 1, [0, 0, 0]) # Turn off LEDs
                        buzzer_client(0.2)  # This
                        time.sleep(0.5)     # is
                        buzzer_client(0.2)  # yet
                        time.sleep(0.5)     # another
                        buzzer_client(0.5)  # cooldown
            else: # The user does not want to play
                led_blink_client(2, 0, [255, 0, 0]) # Pitch lights up in red
                servo_client(0, 20) # Looks up in disdain
                motor_client("lft") # This
                motor_client("lft") # is
                motor_client("lft") # a 
                motor_client("lft") # rotation
                motor_client("fwd") # Runs in the opposite direction
                buzzer_client(1)    # Cries
                motor_client("fwd") # Keeps running
                motor_client("fwd") # Bye
                break
    except rospy.ROSInterruptException:
        print("The program (main_init.py) has just been interrupted !", file=sys.stderr)

