import rospy
import os
from secure_teleop_remote.srv import key, keyRequest, keyResponse


def handle_receive_public_key(req :keyRequest):
    home = os.path.expanduser("~")
    try:
        os.chdir(home + "/Key1")
    except:
        os.mkdir(home + "/Key1")
        os.chdir(home + "/Key1")
    with open("public.pem", "wb") as e:
        e.write(req.key)
    print("成功写入秘钥")
    return keyResponse(True)


def receive_public_key():
    rospy.init_node("get_key")
    s = rospy.Service('exchange', key, handler=handle_receive_public_key)
    print("等待秘钥....")
    rospy.spin()


if __name__ == "__main__":
    receive_public_key()