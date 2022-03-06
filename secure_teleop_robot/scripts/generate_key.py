#!/usr/bin/env python
import os
from Crypto.PublicKey import RSA
from secure_teleop_robot.srv import *
import rospy


def generate_key():

    home = os.path.expanduser("~")
    try:
        os.chdir(home + "/Key0")
    except:
        os.mkdir(home + "/Key0")
        os.chdir(home + "/Key0")

    try:
        f = open("public.pem", "rb")
    except:

        key = RSA.generate(1024)
        private_key = key.export_key()
        public_key = key.publickey().export_key()
        with open("private.pem", "wb") as x:
            x.write(private_key)
        with open("public.pem", "wb") as x:
            x.write(public_key)
        f = open("public.pem", "rb")
    else:

        public_key = f.read()
        f.close()

    return public_key


def send_key(public_key):

    rospy.wait_for_service('exchange')
    try:

        send = rospy.ServiceProxy('exchange', key)
        msg = keyRequest()
        msg.key = public_key
        resp = send(msg)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def send_success():
    rospy.set_param("success", True)


if __name__ == '__main__':
    public_key = generate_key()
    success = send_key(public_key)
    if(success == True):
        print("succeed")
        send_success()
    else:
        print("failed")
