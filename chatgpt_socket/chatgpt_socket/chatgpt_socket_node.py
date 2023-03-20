import os
from comment_msgs.msg import Chat
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import openai

class ChatGPT:
    def __init__(self, api_key):
        openai.api_key = api_key

    def generate_text(self, prompt, comment, length=1000):
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "user", "content": prompt},
                {"role": "assistant", "content": "回答: <motion>HAPPY,NONE</motion><comment>こんにちは！なお♡です。みんなで楽しくトークしましょう。</comment>"},
                {"role": "user", "content": comment},
            ],
        )
        result = ''
        for choice in response.choices:
            result += choice.message.content
        return result

class SpeakROS:
    def __init__(self, ros_installed_setup_bash: str):
        self.ros_installed_setup_bash = ros_installed_setup_bash

    def speak(self, text: str):
        command = "source " + self.ros_installed_setup_bash + " && ros2 run speak_ros test_client \"" + text + "\""
        os.system("/bin/bash -c \"" + command + "\"")

class ChatGPTSocket(Node):
    def __init__(self):
        super().__init__('chatgpt_socket')

        # params
        self.declare_parameter('ros_installed_setup_bash', '~/ws_aivtuber/install/setup.bash')
        self.declare_parameter('text_path', '~/ws_aivtuber/src/ROS2-AI-VTuber-Projects/chatgpt_socket/config/prompt.txt')

        self.ros_installed_setup_bash = self.get_parameter('ros_installed_setup_bash').value
        self.text_path = self.get_parameter('text_path').value

        self.speak_ros  = SpeakROS(os.path.expanduser(self.ros_installed_setup_bash))
        self.chatgpt    = ChatGPT(os.environ['OPENAI_API_KEY'])

        with open(os.path.expanduser(self.text_path), 'r') as f:
            self.prompt = f.read()

        self.subscription_ = self.create_subscription(
            Chat,
            'chat',
            self.listener_callback,
            1)

        self.motion_socket_pub_ = self.create_publisher(String, 'motion_socket', 1)

    def listener_callback(self, msg: Chat):
        author = msg.author
        message = msg.message

        chat = "コメント ({}): {}".format(author, message)
        result = self.chatgpt.generate_text(self.prompt, chat)

        self.get_logger().info("")
        self.get_logger().info("=============================")
        self.get_logger().info("chat: " + chat)
        self.get_logger().info("result: " + result)
        self.get_logger().info("motion: " + result.split('<motion>')[1].split('</motion>')[0])
        self.get_logger().info("=============================")

        # <motion>, <comment>
        motion = result.split('<motion>')[1].split('</motion>')[0]
        comment = result.split('<comment>')[1].split('</comment>')[0]

        self.motion_socket_pub_.publish(String(data=motion))
        # rclpy.spin_once(self, timeout_sec=0.1)
        self.speak_ros.speak(comment)
        # rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    motion_socket = ChatGPTSocket()
    rclpy.spin(motion_socket)
    motion_socket.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
