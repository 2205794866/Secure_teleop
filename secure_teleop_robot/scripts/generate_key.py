#!/usr/bin/env python
import os
from Crypto.PublicKey import RSA
from secure_teleop_robot.srv import *
import rospy


def generate_key():

    #新建存放秘钥的文件夹
    home = os.path.expanduser("~")
    try:
        os.chdir(home + "/Key0")
    except:
        os.mkdir(home + "/Key0")
        os.chdir(home + "/Key0")

    try:
        f = open("public.pem", "rb")
    except:
        #无内置秘钥
        #正在重新生成
        key = RSA.generate(2048)
        private_key = key.export_key()
        public_key = key.publickey().export_key()
        with open("private.pem", "wb") as x:
            x.write(private_key)
        with open("public.pem", "wb") as x:
            x.write(public_key)
        f = open("public.pem", "rb")
    else:
        #读取内置秘钥
        public_key = f.read()
        f.close()

    return public_key


def send_key(public_key):
    #等待秘钥接受
    rospy.wait_for_service('exchange')
    try:
        #发送秘钥
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
        print("秘钥交换成功")
        send_success()
    else:
        print("秘钥交换失败")
