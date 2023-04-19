#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String, Float32MultiArray,Int32MultiArray
from sensor_msgs.msg import Joy
from ControllerBlocks import CPG,Delay
from MORFcontrollers import Motormapping_angle
import sys
import getch
import time
import signal
import numpy as np
def main():
    global motion,speed,pub,count_motion,set_sequence,pump, motion_signal, button_before, ledMode, mssXbutton
    pub = rospy.Publisher('arduino_control', Float32MultiArray, queue_size=1)
    pub_dynamixel = rospy.Publisher('set_position', Int32MultiArray, queue_size=1)
    rospy.init_node('main', anonymous=True)
    rate = rospy.Rate(60) 
    cpg_walk = CPG()
    cpg_walk.set_frequency()
    cpg_breathe = CPG()
    cpg_breathe.set_frequency()
    cpg_lights = CPG()
    cpg_lights.set_frequency()
    mapping = Motormapping_angle()
    TORQUE = 0
    dynamixel_positon = [0]*19 
    motion = "set"
    motion_before = "set"
    motion_change_signal= ""
    speed = "sigma"
    sigma = 0.03
    button_before = [0,0,0,0,0,0,0,0]
    ledMode = 0
    mssXbutton = 0 
    

    breathe_state_0 = True
    breathe_state_1 = True
    time_t0 = time.perf_counter() # seconds
    arduino_control = [0,0,0,0,0,ledMode]
    MOTOR1_DATA = 0  
    MOTOR2_DATA = 0
    cpg_lights_data_0 = 0
    cpg_lights_data_1 = 0
    signal_leg = [0,0,0,0,0]
    count_change = 0
    count_motion = 0
    set_sequence = False
    pump = False

    motion_signal = 0
    

    #----------------------------------------------------------------------------

    # set_alpha and max_alpha must set between [0.001,0.2]. 
    # the breathing freq. will start from set_alpha to max_alpha with change speed of rate_alpha
    # increse alpha: fast breathing  / decreses alpha: slow breathing
    set_alpha = 0.005 
    #set_alpha 0.005 is equal to 10 BPM
    #set_alpha 0.007 is equal to 24 BPM
    alpha = set_alpha
    min_alpha = set_alpha

    max_alpha = 0.011
    #max_alpha 0.011 is equal to 40 BPM
    #max_alpha 0.017 is equal to 60 BPM


    rate_alpha = 0.000009
    # calculate time between set_alpha and max_alpha. ((max_alpha-set_alpha)/rate_alpha)/60

    # set_shif_cpg_breathe and max_shif_cpg_breathe must set between [0,0.3]
    # the deuration will start from set_shif_cpg_breathe to max_shif_cpg_breathe with change speed of rate_cpg_breathe
    # increse shif_cpg_breathe: let less air flow in and let more air flow out/ decreses shif_cpg_breathe: let more air flow in and let less air flow out
    # 0 mean 50% air flow in and 50% air flow out 
    set_shif_cpg_breathe = 0.185 #I feel that less then 19 difforms the air pockets uneven - Henri
    #se 0.08
    shif_cpg_breathe = set_shif_cpg_breathe
    min_shif_cpg_breathe = set_shif_cpg_breathe

    max_shif_cpg_breathe = 0
    # max 0.04
    rate_cpg_breathe = 0.0002
    # calculate time between set_shif_cpg_breathe and max_shif_cpg_breathe. ((max_shif_cpg_breathe-set_shif_cpg_breathe)/rate_cpg_breathe)/60
    #current time from set_shif_cpg_breathe to max_shif_cpg_breathe is 5.55 seconds.

    #----------------------------------------------------------------------------


    while not rospy.is_shutdown():

        rospy.Subscriber('joy', Joy, joy_cb, queue_size=1)

        #----set sequence--------------------------------------------------------------------------------------
        if set_sequence == True:
            if count_motion < 1000:
                motion = "forward"
            elif count_motion < 1200:
                motion = "stop"
            elif count_motion < 2200: 
                motion ="right"
            elif count_motion < 2400:
                motion = "stop"
            elif count_motion < 3400:
                motion = "forward"
            elif count_motion < 3600:
                motion = "stop"
            count_motion +=1
            #print("count_motion: %d"%count_motion)
        #--------------------------------------------------------------------------------------------------------
        

        if motion != "set":
            if motion != "stop" and count_change < 1:
                signal_leg[0] *= count_change
                signal_leg[1] *= count_change
                signal_leg[2] *= count_change
                count_change += 0.005
            dynamixel_positon = mapping.map([signal_leg[0],signal_leg[1],signal_leg[2],signal_leg[3],signal_leg[4]])
            cpg_walk_data = np.array(cpg_walk.update())

        if motion == "set":
            count_change = 0
            sigma = 0.03
            dynamixel_positon = mapping.map([0,0,0,1,1]) 
            cpg_walk = CPG()
            cpg_walk.set_frequency(sigma * np.pi)
        elif motion == "forward":
            if (motion_before != "forward"):
                motion_signal = 1
            signal_leg[0] = cpg_walk_data[0]
            signal_leg[1] = cpg_walk_data[1] 
            signal_leg[2] = -cpg_walk_data[1]
            signal_leg[3] = 1
            signal_leg[4] = 1
        elif motion == "backward":
            if (motion_before != "backward"):
                motion_signal = 2
            signal_leg[0] = -cpg_walk_data[0]
            signal_leg[1] = cpg_walk_data[1] 
            signal_leg[2] = -cpg_walk_data[1]
            signal_leg[3] = 1
            signal_leg[4] = 1
        elif motion == "left":
            if (motion_before != "left"):
                motion_signal = 5
            signal_leg[0] = cpg_walk_data[0]
            signal_leg[1] = cpg_walk_data[1] 
            signal_leg[2] = -cpg_walk_data[1]
            signal_leg[3] = 0.5
            signal_leg[4] = 1
        elif motion == "right":
            if (motion_before != "right"):
                motion_signal = 6
            signal_leg[0] = cpg_walk_data[0]
            signal_leg[1] = cpg_walk_data[1] 
            signal_leg[2] = -cpg_walk_data[1]
            signal_leg[3] = 1
            signal_leg[4] = 0.5
        elif motion == "stop":
            if (motion_before != "stop"):
                motion_signal = 3
            if count_change > 0:
                count_change -= 0.005
                if motion_before_stop == "forward":
                    signal_leg[0] = cpg_walk_data[0] * count_change
                    signal_leg[1] = cpg_walk_data[1] * count_change
                    signal_leg[2] = -cpg_walk_data[1]* count_change
                    signal_leg[3] = 1
                    signal_leg[4] = 1
                elif motion_before_stop == "backward":
                    signal_leg[0] = -cpg_walk_data[0]* count_change
                    signal_leg[1] = cpg_walk_data[1] * count_change
                    signal_leg[2] = -cpg_walk_data[1]* count_change
                    signal_leg[3] = 1
                    signal_leg[4] = 1
                elif motion_before_stop == "left":
                    signal_leg[0] = cpg_walk_data[0]* count_change
                    signal_leg[1] = cpg_walk_data[1] * count_change
                    signal_leg[2] = -cpg_walk_data[1]* count_change
                    signal_leg[3] = 0.5
                    signal_leg[4] = 1
                elif motion_before_stop == "right":
                    signal_leg[0] = cpg_walk_data[0]* count_change
                    signal_leg[1] = cpg_walk_data[1] * count_change
                    signal_leg[2] = -cpg_walk_data[1]* count_change
                    signal_leg[3] = 1
                    signal_leg[4] = 0.5

        #We need this for the light signalling
        motion_before = motion

        #We have to do this, because we don't stop but we slow down the speed, and for that we need to know what we did before stopping
        if motion != "stop":
            motion_before_stop = motion
        



        if speed == "+sigma":
            sigma += 0.0001
            if sigma >= 0.06: sigma = 0.06
            cpg_walk.set_frequency(sigma * np.pi)
        elif speed == "-sigma":
            sigma -= 0.0001
            if sigma <= 0.01: sigma = 0.01
            cpg_walk.set_frequency(sigma * np.pi)


        #print("motion: %s"%motion)
        #print("count_change: %.4f"%count_change)
        #print("sigma: %.4f"%sigma)
        # learning_rate = 0.01
        # forgeting_rate = 0.01
        # if motion == "stop":
        #     state = 0
        # else
        #     state = 1
        # alpha = (learning_rate * state * (sigma + alpha)) + (forgeting_rate * (state-1) * (alpha**2))

        if motion == "set":
            arduino_control = arduino_control = [0,0,250,250,0,ledMode]
            alpha = set_alpha
            shif_cpg_breathe = set_shif_cpg_breathe
            time_t0 = time.perf_counter()
            cpg_breathe = CPG()
            cpg_breathe.set_frequency()
        elif motion == "stop":
            alpha -= rate_alpha
            if alpha <= min_alpha: alpha = min_alpha
            shif_cpg_breathe += rate_cpg_breathe
            if shif_cpg_breathe >= min_shif_cpg_breathe: shif_cpg_breathe = min_shif_cpg_breathe
            cpg_breathe.set_frequency(alpha * np.pi)
        else:
            alpha += rate_alpha
            if alpha >= max_alpha: alpha = max_alpha
            shif_cpg_breathe -= rate_cpg_breathe
            if shif_cpg_breathe <= max_shif_cpg_breathe: shif_cpg_breathe = max_shif_cpg_breathe
            cpg_breathe.set_frequency(alpha * np.pi)


        if pump == True:
            arduino_control = [1,1,250,250,0,ledMode]
            
        if motion != "set":
            
            cpg_breathe_data_0 = np.array(cpg_breathe.update())[0] + shif_cpg_breathe
            cpg_breathe_data_1 = np.array(cpg_breathe.update())[1] + shif_cpg_breathe


              
            cpg_lights_data_0 = (220-(np.array(cpg_breathe.update())[0]+0.4)*270)%255
            cpg_lights_data_1 = (30+(np.array(cpg_breathe.update())[1]+0.4)*270)%255

            if breathe_state_0 == True and cpg_breathe_data_0 >= 0:
                MOTOR1_DATA = 0
                breathe_state_0 = False
                #print("----------------------------------------------------")
                #print(time.perf_counter()-time_t0)
                time_t0 = time.perf_counter()
            elif breathe_state_0 == False and cpg_breathe_data_0 < 0:
                MOTOR1_DATA = 1
                breathe_state_0 = True

            if breathe_state_1 == True and cpg_breathe_data_1 >= 0:
                MOTOR2_DATA = 0
                breathe_state_1 = False
            elif breathe_state_1 == False and cpg_breathe_data_1 < 0:
                MOTOR2_DATA = 1
                breathe_state_1 = True
       
            #if the MOTOR2_DATA = 1 than the second pair will pump air and close the valves
            #if the MOTOR2_DATA = 0 than the pumps will stop and the valves will open
            # this is the same for MOTOR1_DATA 
            # => this logic is implemented in the arduino code, now we are just sending the desired functionality  
            if motion_signal != 0:
                print("---------------------------------------")
                print("motion_signal %.1f" %motion_signal)
                print("---------------------------------------")
            arduino_control  = [MOTOR1_DATA,MOTOR2_DATA,cpg_lights_data_0,cpg_lights_data_1, motion_signal, ledMode]
            motion_signal = 0

            

            #print("cpg_breathe %.4f"% np.array(cpg_breathe.update())[1] )
            #print("cpg_lights_data_0 : %.4f"%cpg_lights_data_0)
            #print("cpg_lights_data_1 : %.4f"%cpg_lights_data_1)
            #print("MOTOR1_DATA %f"%MOTOR1_DATA)
            #print("MOTOR2_DATA %f"%MOTOR2_DATA)
        
        #print("alpha %.4f"%alpha)
        #print("shif_cpg_breathe %.4f"%shif_cpg_breathe)       
        print("ledMode %f"% ledMode)

        dynamixel_control_data = Int32MultiArray()
        dynamixel_control_data.data = dynamixel_positon
        pub_dynamixel.publish(dynamixel_control_data)

        arduino_control_data = Float32MultiArray()
        arduino_control_data.data = arduino_control
        pub.publish(arduino_control_data)
        rate.sleep()

        signal.signal(signal.SIGINT, keyboard_interrupt_handler)   

def joy_cb(msg):
    global motion,speed,count_motion,set_sequence,pump, motion_signal, button_before, ledMode, mssXbutton
    print("buttons message  ")
    print(msg.buttons)
    #print("axis message")
    #print(msg.axes)


    if msg.buttons[7] == 1:
        motion = "set"
        stop_sequence()
    if msg.buttons[1] == 1:
        motion = "stop"
        stop_sequence()
    elif msg.axes[7] == 1:
        motion = "forward"
        stop_sequence()
    elif msg.axes[7] == -1:
        motion = "backward"
        stop_sequence()
    elif msg.axes[6] == 1:
        motion ="left"
        stop_sequence()
    elif msg.axes[6] == -1:
        motion ="right"
        stop_sequence()
    
    #for the X button on the logitech wireless controller
    #if msg.buttons[2] == 1 :
    # Do something

    
    if msg.buttons[2] == 1 and mssXbutton != 1:
        ledMode += 1
        ledMode = ledMode % 2
    mssXbutton = msg.buttons[2]

    if msg.buttons[5] == 1 :
        set_sequence = True
    
    if msg.buttons[4] == 1:
        pump = True
    else:
        pump = False

    if  msg.buttons[3] == 1 :
        speed = "+sigma"
    elif  msg.buttons[0] == 1 :
        speed = "-sigma"
    else:
        speed = "sigma"

    button_before = msg.buttons


def stop_sequence():
    #function to stop the predefined sequence.
    global set_sequence, count_motion
    set_sequence = False
    count_motion = 0

def keyboard_interrupt_handler(keysignal, frame):
    global pub
    arduino_control = [0,0,0,0,0,ledMode]
    arduino_control_data = Float32MultiArray()
    arduino_control_data.data = arduino_control
    pub.publish(arduino_control_data)
    print("closing.")
    exit(0)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
