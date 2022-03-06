#! /usr/bin/env python
import os
import rospy
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist


import Crypto.PublicKey.RSA
import Crypto.Cipher.PKCS1_v1_5
import Crypto.Random


def getPrivate():
    try:
        home = os.path.expanduser("~")
        with open(home + "/Key0/private.pem", "rb") as x:
            private_key = x.read()
    except:
        rospy.signal_shutdown("There is not a Private_key")
    else:
        return private_key


def callback(cipher_text, arg):
    pub = arg[0]
    private_key = arg[1]
    cipher_private = Crypto.Cipher.PKCS1_v1_5.new(
        Crypto.PublicKey.RSA.importKey(private_key))
    msg = cipher_private.decrypt(
        bytearray(cipher_text.data), Crypto.Random.new().read).decode('ASCII')
    print(msg)
    x, z = msg.split(" ")
    # print("x:{}".format(float(x)))
    # print("z:{}".format(float(z)))
    twist = Twist()
    twist.linear.x = float(x)
    twist.angular.z = float(z)
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    pub.publish(twist)


if __name__ == '__main__':
    rospy.init_node('move')
    while rospy.get_param("success", False) != True:
        pass
    private_key = getPrivate()
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("Cypered_cmd_vel", Int16MultiArray, callback,
                     callback_args=(pub, private_key), queue_size=1)
    rospy.spin()
