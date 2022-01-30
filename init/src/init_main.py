#! /usr/bin/env python3

#####################################################################
# Importation of the necessary functions/packages/services/actions. #
#####################################################################
# Packages
from __future__ import print_function
from re import S
from random import randint, choice
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
    """
    Client for the camera service, allowing for recovery of various info (color, thumb state, number of fingers, face distance)
    @param String data (Object to detect (color/hand/face/wheel))
    @return int nb_finger (Number of fingers)
    @return int thumb_state (1 is up, -1 is down)
    @return String color_name (name of the color)
    @return int color_occurence (is the color color_name seen)
    @return int height (Height of the seen color)
    """
    rospy.wait_for_service("camera_service")
    try:
        switch = rospy.ServiceProxy(
            "camera_service", SwitchService
        )  # Use the SwitchService for ease of use
        res = switch(data)  # Returned data according to the switcher
        return (
            res.nb_finger,
            res.thumb_state,
            res.distance,
            res.color_name,
            res.color_occurrence,
            res.height,
        )
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def motor_client(direction, t):
    """
    Client for the motor action server, allowing for movement of the robot for a time "t"
    @param String direction ("fwd" -> forward, "lft" -> left, "rgt" -> right, "bwd" -> backwards)
    @param float t (duration of movement)
    @return bool get_result (bool of action state -> True is done, False is error)
    """
    client = actionlib.SimpleActionClient("motor_action_server", motor.msg.MotorAction)
    client.wait_for_server()
    goal = motor.msg.MotorGoal(order=direction) 
    client.send_goal(goal) # Send movement info
    time.sleep(t) # Sleep for time "t", allowing for movement during "t" seconds
    client.cancel_goal()
    client.wait_for_result()
    return client.get_result()


def servo_client(panningAngle, tiltAngle):
    """
    Client for the servomotor action server, allowing for panning and tilting of the camera
    @param int panningAngle
    @param int tiltAngle
    """
    client = actionlib.SimpleActionClient("servo_action_server", servo.msg.ServoAction)
    client.wait_for_server()
    goal = servo.msg.ServoGoal(panningAngle, tiltAngle) # Send angles
    client.send_goal(goal)


def led_blink_client(time, mode, color):
    """
    Client for the LEDs service, allowing for lightup of the Robot's LEDs
    @param int time (time for LEDs use)
    @param int mode (0 -> blinking, 1 -> wheel, 2 -> static)
    """
    rospy.wait_for_service("led_blink")
    try:
        blink = rospy.ServiceProxy("led_blink", BlinkLED)
        blink.call(BlinkLEDRequest(time, mode, color))
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def buzzer_client(buzzTime):
    """
    Client for the buzzer, allowing for use of the robot's buzzer
    @param float buzzTime (time to buzz for)
    @return bool buzzState (state of the buzzer)
    """
    rospy.wait_for_service("buzzer")
    try:
        buzzerProxy = rospy.ServiceProxy("buzzer", Buzzer)
        resp = buzzerProxy.call(BuzzerRequest(buzzTime))

        if not resp.buzzState == True:
            raise Exception(
                "End of Call failure, returned resp was %f" % resp.buzzerState
            )
        return resp.buzzState

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def ir_callback(button):
    """
    Listener for the IR Remote talker, allowing for use of the remote and the selection of games
    @param string button (what button has been pressed)
    @return int chosen_game_ID (1 -> RPS, 2 -> Wheel, 3 -> Simon)
    """
    global chosen_game_ID
    chosen_game_ID = -1
    msg = button.data # Get the message string
    if msg == "0 (0x16)":
        chosen_game_ID = 0 # None
    if msg == "1 (0x0C)":
        chosen_game_ID = 1 # RPS
    if msg == "2 (0x18)":
        chosen_game_ID = 2 # Wheel
    if msg == "3 (0x5E)":
        chosen_game_ID = 3 # Simon

##############################################################
#  Pitch-specific functions (intialisation, camera, etc...). #
##############################################################
def pitch_init():
    """
    Initialisation function of the robot, setting variables, resetting states and subscribing to topics
    @param none
    @return none
    """
    print("[PITCH - INIT] I'm starting up !")
    rospy.init_node("pitch_client")
    rospy.Subscriber("remote_input", String, ir_callback)
    led_blink_client(0, 2, [0, 0, 0]) # Reset the LEDs
    global user_distance # Distance to user
    user_distance = -1
    global min_treshold_distance # Min Treshold Distance to consider someone in frame
    min_treshold_distance = 40
    global max_treshold_distance # Max Treshold Distance to consider someone in frame
    max_treshold_distance = 100
    global thumb_state  # Thumb state (1 is thumbs up, -1 is thumbs down)
    thumb_state = 0
    global nb_finger    # Number of fingers held up
    nb_finger = -1
    global chosen_game_ID   # Chosen game ID (1 -> RPS, 2 -> Wheel, 3 -> Simon)
    chosen_game_ID = -1
    global game_begin   # State variable used when a game is launched
    game_begin = False
    global choice_done  # State variable used when a game is selected
    choice_done = False


def look_for_user():
    """
    Function using the camera and servos, panning around to find an user.
    @param none
    @return int measured_distance (measured distance to an user's face)
    @return int z (servo pan angle)
    """
    t = 0   # Number of tests
    z = -50 # Starting panning angle
    servo_client(z, -30) # Pan the camera
    rospy.wait_for_service("camera_service")
    while t < 5:  # 5 Tests before validation
        face = camera_client("face") # Look for a face
        measured_distance = face[2]  # Store the measured distance
        print(f"[PITCH - LOOKING FOR USER] Measured Distance : {measured_distance}")
        if measured_distance != -1: # If a face is found
            if measured_distance > max_treshold_distance: # Is it in range ?
                print("[PITCH - LOOKING FOR USER] You're a bit far ! I'm coming closer.") # Too far
                motor_client("fwd",0.5) # Move forward
            if measured_distance < min_treshold_distance: # Is it in range ?
                print("[PITCH - LOOKING FOR USER] Too close ! Let me back away a little bit.") # Too close
                motor_client("bwd",0.5) # Move backward
            else:   # If face in range
                t = t + 1 
        else: # If no face is found
            z = z + 25  # Pan the camera
            if z > 50:
                z = -50 # Go back to starting angle
                motor_client("lft",1.0) # Turn left
            servo_client(z, -30) # Move servo
            print("[PITCH - LOOKING FOR USER] I can't find anyone...")
            t = 0
    return measured_distance, z


def wanna_play():
    """
    Function using the camera to find out if the user wants to play.
    @param none
    @return int thumb_state (1 is up, -1 is down)
    """
    y = 0 # Amount of "yes" seen
    n = 0 # Amount of "no" seen
    rospy.wait_for_service("camera_service")
    while y < 3 and n < 3:  # 3 Tests before validation
        hand = camera_client("hand") # Check for Hand
        nb_finger = hand[0]
        thumb_state = hand[1]
        if nb_finger != -1 and thumb_state != 0: # Fingers are seen and thumb is acknowledged
            if thumb_state == 1: # Thumbs up
                print("[PITCH - PLAY QUERY] You said yes !", y, "times")
                y = y + 1
                n = 0
            if thumb_state == -1:# Thumbs down
                print("[PITCH - PLAY QUERY] You said no..", n, "times")
                n = n + 1
                y = 0 
        else: # Thumbs lost
            print("[PITCH - PLAY QUERY] I can't see your fingers...")
            y = 0
            n = 0
    return thumb_state


def win_dance(neck_angle):
    """
    Function used to make Pitch "dance" after a win.
    @param int neck_angle (angle of the servo at the beginning of the dance)
    @return none
    """
    led_blink_client(6, 1, [255, 0, 255]) # Blink for visual feedback
    servo_client(neck_angle, -80)
    time.sleep(0.2)       # dancing
    servo_client(neck_angle, -30)
    time.sleep(0.2)       # dancing
    servo_client(neck_angle, -80)
    time.sleep(0.2)       # dancing
    servo_client(neck_angle, -30)
    
    motor_client("lft",0.5) # Pitch
    buzzer_client(0.5)      # is
    motor_client("rgt",0.5) # now
    time.sleep(1)           # dancing
    buzzer_client(0.5)      # after 
    motor_client("fwd",0.5) # its
    time.sleep(1)           # win
    buzzer_client(0.5)      # for
    motor_client("bwd",0.5) # about
    time.sleep(1)           # six
    buzzer_client(0.5)      # seconds
    time.sleep(1)           # roughly
    led_blink_client(0, 2, [0, 0, 0]) # Turn off leds
   

def lose(neck_angle):
    """
    Function used to make Pitch react after a loss.
    @param int neck_angle (angle of the servo at the beginning of the dance)
    @return none
    """
    led_blink_client(0, 2, [255, 0, 0]) # Blink red for visual feedback
    buzzer_client(2) # Buzz out of rage
    servo_client(-50, -80)
    time.sleep(0.2)
    servo_client(50, -80)
    time.sleep(0.2)
    servo_client(-50, -80)
    time.sleep(0.2)
    servo_client(neck_angle, -30)
    led_blink_client(0, 2, [0, 0, 0]) # Turn off leds

######################################################
#  Game 1-specific functions (Rock, Paper, Scissors) #
######################################################
def RPS_init():
    """
    Function used to initialize the RPS playground, setting variables.
    @param none
    @return none
    """
    global RPS_moves        # RPS moves and their IDs
    RPS_moves = {1: "Rock", 2: "Paper", 3: "Scissors"}
    global RPS_colors       # RPS moves color and their RGB Arrays
    RPS_colors = {"Rock": [0, 0, 255], "Paper": [0, 255, 0], "Scissors": [255, 0, 0]}
    global RPS_scenarios    # RPS scenarios (Win/Losses/Ties)
    RPS_scenarios = {  # Win == True | Tie == None | Loss == False
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
    global RPS_gameSet # Bool varible to state if the game is set (Won or Lost)
    RPS_gameSet = False
    global RPS_playerReady # Int varible to state if the player is ready
    RPS_playerReady = 1
    print("[PITCH - RPS - INIT] Wait a moment ! I'm setting up the RPS playground.")


def RPS_pitch_move():
    """
    Function used by Pitch to select a RPS move, and light up
    @param none
    @return int pitch_move (ID of the RPS move, relative to the RPS_moves dictionnary)
    """
    pitch_move = randint(1, 3)
    indicator_color = RPS_colors[RPS_moves[pitch_move]]
    print(
        f"[PITCH - RPS - MOVE CHOICE] I'm going with... {RPS_moves[pitch_move]} ! I'll light up in {indicator_color}."
    )
    led_blink_client(0, 2, indicator_color)
    return pitch_move


def RPS_camera_analysis():
    """
    Function used by Pitch to check the user's move
    @param none
    @return int user_move (ID of the RPS move, relative to the RPS_moves dictionnary)
    """
    t = 0
    while t < 3:  # 3 Tests
        hand = camera_client("hand")
        nb_finger = hand[0]
        thumb_state = hand[1]
        if nb_finger >= 4: # 4 or more (for edge cases)
            user_move = 2 # Paper
            t = t + 1
        elif nb_finger >= 2 and nb_finger < 4: # Between 2 and 3 fingers
            user_move = 3 # Scissors
            t = t + 1
        elif nb_finger > -1 and nb_finger < 2: # Zero to one fingers
            user_move = 1 # Rock
            t = t + 1
        else:
            t = 0
    print(f"[PITCH - RPS - MOVE CHECK] Oh, so your move is... {RPS_moves[user_move]} !")
    return user_move


def RPS_game_state(pitch_move, user_move):
    """
    Function used to check the game state 
    @param int pitch_move (ID of Pitch's move)
    @param int user_move (ID of the user's move)
    @return bool gameSet (boolean representative of the game being set or not)
    @return bool/None State (variable representative of the game state : win/tie/loss)
    """
    state = RPS_scenarios[(pitch_move, user_move)]
    if state == None:
        print("[PITCH - RPS - GAME STATE MANAGER] It's a tie ! Let's go again.")
        gameSet = False
    else:
        gameSet = True
    return (gameSet, state)
    

######################################
#  Game 2-specific functions (Wheel) #
######################################
def Whl_init():
    """
    Function used to initialize the Wheel playground, setting variables.
    @param none
    @return none
    """
    global red # RGB code and string for the color red
    red = [[255, 0, 0], "red"]
    global green
    green = [[0, 255, 0], "green"]
    global blue
    blue = [[0, 0, 255], "blue"] 
    global Whl_playerReady
    Whl_playerReady = 1
    print("[PITCH - WHEEL - INIT] Wait a moment ! I'm setting up the Wheel playground.")


def Whl_color():
    """
    Function used to guess a random color.
    @param none
    @return array color (array containing the RGB code and name string)
    """
    color = choice([red, green, blue])
    print(
        f"[PITCH - WHEEL - COLOR CHOICE] I'm going with... {color[1]} ! I'll light up in {color[1]}."
    )
    return color


def Whl_camera_analysis():
    """
    Function used by Pitch to check the wheel color
    @param none
    @return string color (string of the corresponding color)
    """
    t = 0
    rospy.wait_for_service("camera_service")
    while t < 3:  # 3 Tests
        wheel = camera_client("wheel")
        found = wheel[3] # RGB
        if found == "none": # If no color is found
            t = 0
        else:
            t = t + 1
        color = found # After making sure (several iterations)
    print("[PITCH - WHEEL - COLOR CHECK] Oh, I see ... ", color, " !")
    return color


######################################################
#  Game 3-specific functions (Simon) #
######################################################
def Smn_init():
    """
    Function used to initialize the Simon Says playground, setting variables.
    @param none
    @return none
    """
    global red
    red = [[255, 0, 0], "red"]
    global green
    green = [[0, 255, 0], "green"]
    global blue
    blue = [[0, 0, 255], "blue"] 
    global Smn_playerReady
    Smn_playerReady = 1
    print("[PITCH - SIMON - INIT] Wait a moment ! I'm setting up the Simon playground.")


def Smn_ready_steady():
    """
    Function used to do a small countdown by buzzing.
    @param none
    @return none
    """
    buzzer_client(0.2)  # This
    time.sleep(0.5)     # is
    buzzer_client(0.2)  # a
    time.sleep(0.5)     # small
    buzzer_client(0.5)  # countdown

def Smn_color():
    """
    Function used to decide on a random color.
    @param none
    @return array color (array containing the RGB code and name string)
    """
    color = choice([red, green, blue])
    print("[PITCH - SIMON - COLOR CHOICE] Please show me something that is", color[1], "!")
    return color


def Smn_camera_analysis(color):
    """
    Function used to see the color that is being held up by the user, and comparing it to the desired color.
    @param string color
    @return bool res (array containing the RGB code and name string)
    """
    t = 0
    rospy.wait_for_service("camera_service")
    while t < 3:  # 3 Tests
        color_seen = camera_client(color[1])
        found = color_seen[4]
        if found:
            res = True
        else:
            res = False
        t = t + 1
    return res


#######################################################
# MAIN ROBOT CALLS, LOGIC AND LOOPS (execute as main) #
#######################################################
if __name__ == "__main__":
    try:
        pitch_init()
        print("[PITCH - INIT] Ready to go !")
        while True: # Infinite Loop, after initialisation
            while ( # Loop while the user is not within range (tresholds) or not found (-1)
                user_distance == -1
                or user_distance > max_treshold_distance
                or user_distance < min_treshold_distance
            ):
                user_distance, neck_angle = look_for_user() # Register the current user distance
                if user_distance == -1: # If no user is found nearby,
                    servo_client(randint(-50, 50), -30) # randomly move servos
            print("[PITCH - LOOKING FOR USER] Found someone !")
            led_blink_client(0, 2, [0, 255, 0]) # Blink Green to have visual feedback
            buzzer_client(0.2) # Buzz for sound feedback
            led_blink_client(0, 2, [0, 0, 0]) # Turn off the LEDs
            thumb_state = wanna_play() # Ask the found user if they want to play
            
            if thumb_state == 1: # If the user wants to play (thumbs up)
                play_game = True
                while play_game:
                    choice_done = False
                    led_blink_client(0, 2, [255, 255, 255]) # Blink White to have visual feedback
                    buzzer_client(0.2)  # Buzz for sound feedback
                    print("[PITCH - GAME QUERY] What game do you want to play ? Use the IR remote to tell me !")
                    while  choice_done == False:  # Forever loop for query
                        
                        if chosen_game_ID == 0: # ID 0 - Cancel the current query
                            play_game = False
                            choice_done = True
                            chosen_game_ID = -1
                            print("[PITCH - GAME QUERY] Game query cancelled. Changed your mind ?")
                        
                        ###########
                        ### RPS ###
                        ###########
                        elif chosen_game_ID == 1: # ID 1 - RPS
                            choice_done = True
                            led_blink_client(0, 2, [255, 0, 255]) # Blink Purple to show selection
                            RPS_init() # Initialize the RPS playground.
                            chosen_game_ID = -1 # Cancel chosen_game_ID, as unused as of now
                            while RPS_playerReady == 1: # Player is considered ready as they enter the game
                                while not RPS_gameSet:  # Loop on ties
                                    buzzer_client(0.2)  # This
                                    time.sleep(0.5)     # is
                                    buzzer_client(0.2)  # a
                                    time.sleep(0.5)     # small
                                    buzzer_client(0.5)  # countdown
                                    user_move = RPS_camera_analysis() # See what the user chose
                                    pitch_move = RPS_pitch_move()     # Generate a random choice for the robot
                                    (RPS_gameSet, state) = RPS_game_state(pitch_move, user_move) # Use game logic to see who won
                                if state:  # Pitch won
                                    print("[PITCH - RPS - RESULTS MANAGER] Yes, I won ! Looks like I'm the boss here !")
                                    win_dance(neck_angle)
                                else: # Pitch lost
                                    print("[PITCH - RPS - RESULTS MANAGER] Whoops, looks like I lost. I'll let you have this one.") # Sore loser
                                    lose(neck_angle)
                                RPS_playerReady = 0
                                RPS_gameSet = False
                                print("[PITCH - RPS - RESULTS MANAGER] Wanna play again ?")
                                #% Use the wanna_play function
                                while RPS_playerReady == 0: # Wait for user feedback
                                    RPS_playerReady = wanna_play()
                                             
                        ###########
                        ## Wheel ##
                        ###########                       
                        elif chosen_game_ID == 2: # ID 2 - Wheel 
                            choice_done = True
                            led_blink_client(0, 2, [0, 255, 255]) # Blink Turquoise to show selection
                            chosen_game_ID = -1 # Cancel chosen_game_ID, as unused as of now
                            Whl_init() # Initialize the RPS playground
                            while Whl_playerReady == 1: # Player is considered ready as they enter the game
                                color = Whl_color()
                                led_blink_client(0, 2, color[0])
                                found = Whl_camera_analysis()
                                if found == color[1]:
                                    print("[PITCH - WHEEL - RESULTS MANAGER] I guessed right !")
                                    win_dance(neck_angle)
                                else:
                                    print("[PITCH - WHEEL - RESULTS MANAGER] Well, looks like I was wrong.")
                                    lose(neck_angle)
                                Whl_playerReady = 0
                                time.sleep(5)
                                print("[PITCH - WHEEL - RESULTS MANAGER] Wanna play again ?")
                                #% Use the wanna_play function
                                while Whl_playerReady == 0: # Wait for user feedback
                                    Whl_playerReady = wanna_play()
                        
                        ###########
                        ## Simon ##
                        ###########     
                        elif chosen_game_ID == 3: # ID 3 - Simon
                            choice_done = True
                            led_blink_client(0, 2, [0, 255, 255]) # Blink Turquoise to show selection
                            chosen_game_ID = -1 # Cancel chosen_game_ID, as unused as of now
                            Smn_init() # Initialize the RPS playground
                            while Smn_playerReady == 1: # Player is considered ready as they enter the game
                                timer = 15 #seconds to find color
                                found = False
                                Smn_ready_steady()
                                color = Smn_color()
                                led_blink_client(timer, 0, color[0])
                                found = Smn_camera_analysis(color)
                                
                                if found:
                                    print("[PITCH - SIMON - RESULTS MANAGER] Well done !")
                                    win_dance(neck_angle)
                                else:
                                    print("[PITCH - SIMON - RESULTS MANAGER] Too late, you lose!")
                                    lose(neck_angle)
                                Smn_playerReady = 0
                                time.sleep(5)
                                print("[PITCH - SIMON - RESULTS MANAGER] Wanna play again ?")
                                #% Use the wanna_play function
                                while Smn_playerReady == 0: # Wait for user feedback
                                    Smn_playerReady = wanna_play()           

            else: # The user does not want to play
                led_blink_client(2, 0, [255, 0, 0]) # Pitch lights up in red
            servo_client(0, -80)
            led_blink_client(0, 2, [0, 0, 0])
            break
    except rospy.ROSInterruptException:
        print("The program (main_init.py) has just been interrupted !", file=sys.stderr)
