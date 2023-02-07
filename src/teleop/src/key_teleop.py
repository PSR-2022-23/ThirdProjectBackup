#!/usr/bin/env python3

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from functools import partial
import rospy
from geometry_msgs.msg import Twist
import sys, select, os
import tty, termios
import tkinter as tk
from tkinter import ttk
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

LIN_VEL_INCREMENT = 0.05
ANG_VEL_INCREMENT = 0.05

WAFFLE_MAX_LIN_VEL = 0.90
WAFFLE_MAX_ANG_VEL = 2.00

LIN_VEL_STEP_SIZE = 0.025
ANG_VEL_STEP_SIZE = 0.025

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x
w/x : increase/decrease linear velocity ( Max vel : ~ 0.90)
a/d : increase/decrease angular velocity ( Max vel : ~ 2.00)
space key, s : force stop
CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(TARGET_LINEAR_VEL, TARGET_ANGULAR_VEL):
    return "currently:\tlinear vel %s\t angular vel %s " % (TARGET_LINEAR_VEL,TARGET_ANGULAR_VEL)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    return constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)


def checkAngularLimitVelocity(vel):

    return   constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
#make car go vrum
def keyBinding(event):
    global TARGET_ANGULAR_VEL, TARGET_LINEAR_VEL, CONTROL_ANGULAR_VEL, CONTROL_LINEAR_VEL

    # Get the key pressed either from an event (when the keybinding is triggered by an event) or from a string (when the function is called with a string)
    key = repr(event.char).replace("\'", "") if type(event) != str else event

    # If key is 'q', print "Quitting..." and exit the program
    if key == 'q':
        print("Quitting...")
        pub.publish(Twist())
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        sys.exit(0)

    # If key is 'w', increase the target linear velocity
    elif key == 'w':
        TARGET_LINEAR_VEL = checkLinearLimitVelocity(TARGET_LINEAR_VEL + LIN_VEL_STEP_SIZE)
    # If key is 's', decrease the target linear velocity
    elif key == 's':
        TARGET_LINEAR_VEL = checkLinearLimitVelocity(TARGET_LINEAR_VEL - LIN_VEL_STEP_SIZE)
    # If key is 'a', increase the target angular velocity
    elif key == 'a':
        TARGET_ANGULAR_VEL = checkAngularLimitVelocity(TARGET_ANGULAR_VEL + ANG_VEL_STEP_SIZE)
    # If key is 'd', decrease the target angular velocity
    elif key == 'd':
        TARGET_ANGULAR_VEL = checkAngularLimitVelocity(TARGET_ANGULAR_VEL - ANG_VEL_STEP_SIZE)
    # If key is ' ', set both target linear and angular velocity to zero
    elif key == ' ':
        TARGET_LINEAR_VEL = TARGET_ANGULAR_VEL = CONTROL_LINEAR_VEL = CONTROL_ANGULAR_VEL = 0.0
    # If the target angular velocity is negative, increase it
    elif TARGET_ANGULAR_VEL < 0:
        TARGET_ANGULAR_VEL = checkAngularLimitVelocity(TARGET_ANGULAR_VEL + ANG_VEL_STEP_SIZE)
    # If the target angular velocity is positive, decrease it
    elif TARGET_ANGULAR_VEL > 0:
        TARGET_ANGULAR_VEL = checkAngularLimitVelocity(TARGET_ANGULAR_VEL - ANG_VEL_STEP_SIZE)
    # If the target linear velocity is negative, increase it
    elif TARGET_LINEAR_VEL < 0:
        TARGET_LINEAR_VEL = checkLinearLimitVelocity(TARGET_LINEAR_VEL + LIN_VEL_STEP_SIZE)
    # If the target linear velocity is positive, decrease it
    elif TARGET_LINEAR_VEL > 0:
        TARGET_LINEAR_VEL = checkLinearLimitVelocity(TARGET_LINEAR_VEL - LIN_VEL_STEP_SIZE)
    # Round target linear velocity to 3 decimal places
    TARGET_LINEAR_VEL = round(TARGET_LINEAR_VEL, 3)
    # Round target angular velocity to 2 decimal places
    TARGET_ANGULAR_VEL = round(TARGET_ANGULAR_VEL, 2)

    CONTROL_LINEAR_VEL = round(makeSimpleProfile(CONTROL_LINEAR_VEL, TARGET_LINEAR_VEL, LIN_VEL_INCREMENT), 3)
    CONTROL_ANGULAR_VEL = round(makeSimpleProfile(CONTROL_ANGULAR_VEL, TARGET_ANGULAR_VEL, ANG_VEL_INCREMENT), 2)

    twist = Twist()
    twist.linear.x = CONTROL_LINEAR_VEL
    twist.angular.z = CONTROL_ANGULAR_VEL
    pub.publish(twist)

#Big thanks to CGPT
if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot3_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    turtlebot3_model = rospy.get_param("model", "waffle_pi")

    TARGET_LINEAR_VEL   = 0.0
    TARGET_ANGULAR_VEL  = 0.0
    CONTROL_LINEAR_VEL  = 0.0
    CONTROL_ANGULAR_VEL = 0.0

window = tk.Tk()
window.title("Teleop")
window.geometry("400x400")

lbl1 = tk.Label(window, text=f"Linear Velocity: {TARGET_LINEAR_VEL}\n\t")    
lbl2 = tk.Label(window, text=f"Angular Velocity: {TARGET_ANGULAR_VEL}\n\t")
lbl3 = tk.Label(window, text=msg)
window.bind('<Key>', keyBinding)

def updateLabel():
    
    global TARGET_LINEAR_VEL, TARGET_ANGULAR_VEL
    
    linear = "Moving " + ("forward" if TARGET_LINEAR_VEL > 0 else "in reverse") if TARGET_LINEAR_VEL != 0 else ""
    angular = "Turning " + ("left" if TARGET_ANGULAR_VEL > 0 else "right") if TARGET_ANGULAR_VEL != 0 else ""
    
    lbl1.config(text=f"Linear Velocity: {TARGET_LINEAR_VEL}\n\t{linear}")
    lbl2.config(text=f"Angular Velocity: {TARGET_ANGULAR_VEL}\n\t{angular}")
    window.after(200, updateLabel)
    keyBinding("")

window.after(600, updateLabel)

lbl1.pack()
lbl2.pack()
lbl3.pack()

window.pack_propagate(False)
window.mainloop()