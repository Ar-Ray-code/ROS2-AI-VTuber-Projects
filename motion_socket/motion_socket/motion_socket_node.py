import os
from std_msgs.msg import String
import rclpy
from rclpy.node import Node

# topic -----------------------------------
# - std_msgs/String
# (format): "<motion>`face`,`motion`</motion>"
# - face : NONE, ANGRY, HAPPY, SAD
# - motion : NONE, HAND_WAVE(hello!), TAI_CHI (Pose), WIPE(hmm...)

# example -----------------------------------
#
# <motion>NONE,HAND_WAVE</motion>
# <motion>NONE,NONE</motion>
# <motion>ANGRY,NONE</motion>
# <motion>HAPPY,HAND_WAVE</motion>
# <motion>SAD,WIPE</motion>
# <motion>NONE,WIPE</motion>
#
# $ ros2 topic pub --once /motion_socket std_msgs/msg/String "{data : '<motion>NONE,HAND_WAVE</motion>'}"


class MotionSocket(Node):
    def __init__(self):
        super().__init__('motion_socket')

        self.subscription_ = self.create_subscription(
            String,
            'motion_socket',
            self.listener_callback,
            10)


    def listener_callback(self, msg):
        # remove <motion> and </motion> and split by ','
        data = msg.data.replace('<motion>', '').replace('</motion>', '').split(',')
        face = data[0]
        motion = data[1]

        self.send_using_socket(face, motion)

    def send_using_socket(self, face: str, motion: str):

        if face == 'NONE':
            face_command = 0
        elif face == 'ANGRY':
            face_command = 1
        elif face == 'HAPPY':
            face_command = 2
        elif face == 'SAD':
            face_command = 3
        else:
            face_command = 0

        # curl
        os.system("curl http://localhost:5000/face_emotion -X PUT -d '" + str(face_command) + "'")

        if motion == 'NONE':
            motion_command = ""
        elif motion == 'HAND_WAVE':
            motion_command = "hand_wave_enable"
        elif motion == 'WIPE':
            motion_command = "wipe_enable"
        else:
            motion_command = ""

        if motion_command != "":
            os.system("curl http://localhost:5000/" + motion_command + " -X PUT -d '1'")
        else:
            pass


def main(args=None):
    rclpy.init(args=args)
    motion_socket = MotionSocket()
    rclpy.spin(motion_socket)
    motion_socket.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
