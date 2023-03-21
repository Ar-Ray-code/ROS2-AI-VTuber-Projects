# ROS2-AI-VTuber-Projects

ROS2系AI VTuberのプロジェクトです。

![image](https://user-images.githubusercontent.com/67567093/226523573-3e97ead6-b7da-46f9-bdf4-128b2c350bc5.png)

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
rosdep install -i --from-path src --rosdistro humble -y
colcon build --symlink-install
```

### OBS install

[obsproject.com](https://obsproject.com/wiki/install-instructions#linux)

```bash
sudo apt install -y v4l2loopback-dkms

sudo add-apt-repository ppa:obsproject/obs-studio # enter `Enter` key to continue
sudo apt update
sudo apt install -y obs-studio
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
