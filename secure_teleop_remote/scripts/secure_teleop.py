#!/usr/bin/env python

#
# 使用 RSA 加密协议对传输信息进行加密
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

import rospy
from std_msgs.msg import Int64MultiArray
from geometry_msgs.msg import Twist
import sys, select, os

if os.name == "nt":
    import msvcrt
else:
    import tty, termios

# import RSA 加密库 pycryptodome name Crypto

import Crypto.PublicKey.RSA
import Crypto.Cipher.PKCS1_v1_5
# import Crypto.Random


BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

start_msg = """
Start to connect and get the Public_key
-----------------------------------------
"""

end_msg = """
Got the Public_key!
----------------------------------
"""

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""


def getKey():
    if os.name == "nt":
        if sys.version_info[0] >= 3:
            return msvcrt.getch().decode()
        else:
            return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (
        target_linear_vel,
        target_angular_vel,
    )


def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
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
    if turtlebot3_model == "burger":
        vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
        vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
        vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel


def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
        vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
        vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
        vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel






# 加载公钥
def getPublicKey():
    try:
        home = os.path.expanduser("~")
        with open(home + "/Key1/public.pem", "rb") as x:
            public_key = x.read()
    except:
      rospy.signal_shutdown("There is not a Public_key")
    else:
        return public_key

# 进行加密
def Encrypt(msg, public_key):
    # recipient_key = RSA.import_key(public_key)
    # session_key = get_random_bytes(16)
    # cipher_rsa = PKCS1_OAEP.new(recipient_key)
    # enc_session_key = cipher_rsa.encrypt(session_key)
    # cipher_aes = AES.new(session_key, AES.MODE_EAX)
    # ciphertext , tag = cipher_aes.encrypt_and_digest(msg)
    cipher_public = Crypto.Cipher.PKCS1_v1_5.new(Crypto.PublicKey.RSA.importKey(public_key))
    cipher_text = cipher_public.encrypt(msg)
    return cipher_text

if __name__ == "__main__":
    if os.name != "nt":
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("secure_teleop")
    while rospy.get_param("success", False) != True:
        pass
    # pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    pub = rospy.Publisher("Cypered_cmd_vel", Int64MultiArray, queue_size=20)

    turtlebot3_model = rospy.get_param("model", "burger")

    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_angular_vel = 0.0

    try:
        print(start_msg)
        public_key = getPublicKey()
        print(end_msg)
        print(msg)
        while 1:
            key = getKey()
            if key == "w":
                target_linear_vel = checkLinearLimitVelocity(
                    target_linear_vel + LIN_VEL_STEP_SIZE
                )
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == "x":
                target_linear_vel = checkLinearLimitVelocity(
                    target_linear_vel - LIN_VEL_STEP_SIZE
                )
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == "a":
                target_angular_vel = checkAngularLimitVelocity(
                    target_angular_vel + ANG_VEL_STEP_SIZE
                )
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == "d":
                target_angular_vel = checkAngularLimitVelocity(
                    target_angular_vel - ANG_VEL_STEP_SIZE
                )
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            elif key == " " or key == "s":
                target_linear_vel = 0.0
                control_linear_vel = 0.0
                target_angular_vel = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))
            else:
                if key == "\x03":
                    break

            if status == 20:
                print(msg)
                status = 0

            twist = Twist()

            control_linear_vel = makeSimpleProfile(
                control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE / 2.0)
            )
            twist.linear.x = control_linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(
                control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE / 2.0)
            )
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_vel
            str_msg = str(twist.linear.x) + " " + str(twist.angular.z)
            cipher_msg = Int64MultiArray()
            cipher_msg.data = list(Encrypt(str_msg.encode('ASCII'), public_key))
            # print(cipher_msg.data)
            pub.publish(cipher_msg)

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        str_msg = str(twist.linear.x) + " " + str(twist.angular.z)
        cipher_msg = Int64MultiArray()
        cipher_msg.data = list(Encrypt(str_msg.encode('UTF-8'), public_key))
        pub.publish(cipher_msg)

    if os.name != "nt":
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
