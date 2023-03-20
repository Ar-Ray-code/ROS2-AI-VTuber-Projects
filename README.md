# ROS2-AI-VTuber-Projects

ROS2系AI VTuberのプロジェクトです。

<br>

## Webots Install

- [Webots install (Linux)](https://cyberbotics.com/doc/guide/installation-procedure#installing-the-debian-package-with-the-advanced-packaging-tool-apt)

### パッケージのインストール

```bash
mkdir -p ~/ws_aivtuber/src
cd ~/ws_aivtuber/src
git clone https://github.com/Ar-Ray-code/ROS2-AI-VTuber-Projects.git -b main
vcs import . < ROS2-AI-VTuber-Projects/speak_ros.repos

cd ~/ws_aivtuber
rosdep install -i --from-path src --rosdistro foxy -y
colcon build --symlink-install
```

<br>

## 実行方法

```bash
ros2 launch aivtuber_bringup aivtuber_nao.launch.py youtube_id:="XXXX"
```

<br>

## パッケージ一覧

- comment_msgs : YouTubeコメントのメッセージ
- motion_socket : Webotsのモーションを解釈してcurlで送るノード
- youtube_comment : YouTubeコメントを取得するノード
- chatgpt_ros2 : ChatGPTのROS2パッケージ

## Reference

- [Webots](https://cyberbotics.com/)
