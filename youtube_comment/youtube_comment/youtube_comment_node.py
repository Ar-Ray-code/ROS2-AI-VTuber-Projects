import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pytchat
import time
import random
from collections import deque

class COMMENT:
    def __init__(self, author, message):
        self.author = author
        self.message = message

def get_comment(livechat_objects, MAX_BUFFER_SIZE) -> COMMENT:
    chatdata = livechat_objects.get()

    comment_buffer = []
    for c in chatdata.items:
        comment_buffer.append(c)

    unique_comments = list(set(comment_buffer))
    num_of_comments_to_display = min(len(unique_comments), MAX_BUFFER_SIZE)
    if num_of_comments_to_display > 0:
        selected_comment = random.choice(unique_comments[:num_of_comments_to_display])
        return COMMENT(selected_comment.author.name, selected_comment.message)

    return COMMENT("", "")

class ChatPublisher(Node):
    def __init__(self):
        super().__init__('chat_publisher')

        # 動画IDのパラメータを取得
        self.declare_parameter('video_id', "")
        video_id = self.get_parameter('video_id').value

        # PytchatCoreオブジェクトの取得
        self.livechat = pytchat.create(video_id=video_id)

        # バッファの初期化
        self.MAX_BUFFER_SIZE = 100
        self.comment_buffer = deque(maxlen=self.MAX_BUFFER_SIZE)

        # ROS 2のトピックの設定
        self.publisher_ = self.create_publisher(String, 'chat', 10)

        # 1秒ごとにコメントを取得し、ROS 2のトピックに送信する
        self.timer = self.create_timer(1.0, self.publish_chat)

    def publish_chat(self):
        # チャットデータの取得

        comment = get_comment(self.livechat, self.MAX_BUFFER_SIZE)
        # print(comment.author, comment.message)

        # publish
        self.publisher_.publish(String(data=f"{comment.author} {comment.message}"))


def main(args=None):
    rclpy.init(args=args)
    chat_publisher = ChatPublisher()
    rclpy.spin(chat_publisher)
    chat_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
