# darknet_ros
### Raspberry Pi 4 + YOLOv3 [ Real-Time Object Detection for ROS ]

<br><br><br>

## :hammer: 프로젝트 소개
kobuki+Raspberry Pi 4 에서 실시간으로 카메라 영상 노드를 전송하여 Desktop에서 yolov3를 실행하는 프로젝트

<br><br>

## ⚙️ 개발환경

### :turtle: Common 
|   **Title**|   **Description**   |
|:--------   |       :-------------|
|ROS          |Noetic Ninjemys     |
|OpenCV       |4.7.0 version       |
|Python       |3.8.10 version      |


<br>


### :robot: Robot
|   **Title**|   **Description**   |
|:--------   |       :-------------|
|Robot       |Kobuki        (yujinrobot)     |
|Camera      |Kinect        (Azure Dk)       |
|OS          |Raspbain Buster  (Pi 4 Model B)      |

<br>

### :computer: Desktop
|   **Title**|   **Description**   |
|:--------   |       :-------------|
|OS          |Ubuntu 20.04 LTS     |
---

<br><br><br>


## :pushpin: Install

<br>

### :strawberry: Raspberry Pi
#### ** - Ros에서 실시간 이미지를 전송에 필요한 python 코드를 실행시킬 패키지 생성**

<br>

#### :one: Create Package
```
$ cd camera_ws/src
$ catkin_create_pkg camera_pkg rospy
```

<br>

#### :two: Create Package
```
$ cd camera_ws/src
$ catkin_create_pkg camera_pkg rospy
```

