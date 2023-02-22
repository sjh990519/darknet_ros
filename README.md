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
#### ** Ros에서 실시간 이미지를 전송에 필요한 python 코드를 실행시킬 패키지 생성**

<br>

### :scroll: Worksapce tree
```
py_test
├── build
├── devel
└── src
    └── test_pkg
        └── src
            └── client.py

```


#### :one: Create Package
```
$ cd py_test/src
$ catkin_create_pkg test_pkg rospy
```

<br>

#### :two: Source code
- scripts 디렉토리에 넣지 안고 c++ 코드처럼 "src" 디렉토리에 만들어서 사용했다.
```
$ cd py_test/src/test_pkg
$ nano publisher.py
```

### :notebook: Publisher.py 
```
#! /usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def image_publisher():
    
    # 대기열이 1인 "/camera/iamge" 라는 노드
    pub = rospy.Publisher('/camera/image', Image, queue_size=1)
    rospy.init_node('image_publisher', anonymous=True)
    
    # 카메라의 이미지를 초당 60프레임으로 캡쳐 
    rate = rospy.Rate(60)

    # video0 번으로 잡혀있는 카메라의 프레임을 캡쳐
    cap = cv2.VideoCapture(0, cv2.CAP_V4L)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            try:
                msg = bridge.cv2_to_imgmsg(frame, "bgr8")
                pub.publish(msg)
            except CvBridgeError as e:
                print(e)
        rate.sleep()

if __name__ == '__main__':
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass

```



