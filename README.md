# darknet_ros
### Raspberry Pi 4 + YOLOv3 [ Real-Time Object Detection for ROS ]

<br><br><br>

## :hammer: 프로젝트 소개
kobuki+Raspberry Pi 4 에서 실시간으로 카메라 영상 노드를 전송하여 Desktop에서 yolov3를 실행하는 프로젝트

<br>

### 1. opencv를 활용하여 실시간 이미지를 전송하는 이유
- 라즈베리 파이에 카메라를 바로 실행하여 실행한 노드를 가지고 darknet_ros를 실행 시 
```
[ERROR] [1675525011.955618390]: Image message encoding provided is not mono8, mono16, bgr8, bgra8, rgb8 or rgba8.
```
- 위와 같은 에러가 발생
- 에러의 내용으로는 제공된 인코딩의 형식이 Yolo와 맞지 않아 발생하는 에러이다.
- 따라서 OpenCV 이미지를 ROS를 통해 게시할 ROS 형식으로 변환하는 방법을 사용하면 에러가 해결된다.

<br>

### 2. 라즈베리 파이에서 바로 Yolo를 실행하지 않는 이유
- 직접 실행시 라즈베리 파이의 물리적인 성능의 한계가 있기 때문이다.
- 따라서 추후 교수님께 받은 Jetson Nano 보드를 활용하여 직접 로봇에서 돌려 볼 예정이다.


<br><br>

## ⚙️ 개발환경

### :turtle: Common 
|   **Title**|   **Description**   |
|:--------   |       :-------------|
|ROS          |Noetic Ninjemys     |
|OpenCV       |4.7.0 version       |
|Python       |3.8.10 version      |

<br>

- Dekstop 
- .bashrc 
```
export ROS_MASTER_URI=http://192.168.0.157:11311
export ROS_HOSTNAME=192.168.0.157
```

<br>

- Raspberry Pi
- .bashrc 
```
export ROS_HOSTNAME=192.168.0.162
export ROS_MASTER_URI=http://192.168.0.157:11311
```

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
|Architecher |x86_64               |
|GPU         |GeForce GTX 1060 3GB |
|CUDA        |11.2.2               |
|cuDNN       |8.1.1                |
---

<br><br><br>


## :pushpin: Install

<br>

### :strawberry: Raspberry Pi
#### ⚙️ Pi OverClock
#### 기본성능으로 노드 전송 시 너무 느려 오버클록을 하여 최대효울을 낼 수 있도록 한다.

<br>

```
$ sudo apt update && sudo apt upgrade && sudo rpipdate
$ sudo reboot
$ sudo nano /boot/config.txt
```

<br>

- 스크롤을 내린 후 [pi4] 항목을 찾은 후 그 아래에 입력
```
over_voltage=6
arm_freq=2147
gpu_freq=700
temp_limit=75
```
1. over_voltage : 2 마다 0.05V의 전압을 더 사용 한다.
2. CPU 클록을 2.14GHz로 늘린다. (최대 2,147GHz)
3. GPU 클록울 700MHz로 늘린다. (최대 750MHz)
4. 파이 온도가 75도가 되면 클록과 전압을 기본으로 회귀한다. (과열방지)


<br>

---

<br>

###  📌 Ros에서 실시간 이미지를 전송에 필요한 python 코드를 실행시킬 패키지 생성

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


### :one: Create Package "publisher"
```
$ cd py_test/src
$ catkin_create_pkg test_pkg rospy
```

<br>

### :two: Source code
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

<br>

####  [Error]   #! /usr/bin/env python3 작성하지 않을 시
```
ImportError: libopencv_imgcodecs.so.3.2: cannot open shared object file: No such file or directory
```
- 위와 같은 에러가 발생
- python의 디렉토리 위치를 확인 후 디렉토리에 맞게 
- 반드시 첫번째 줄에 #! /usr/bin/env python3를 작성한다.

<br>

####  [Error]   cap = cv2.VideoCapture(0) 로 작성 시
```
gstreamer warning: embedded video playback halted; module v4l2src0 reported: could not read from resource.
```
- 위와 같은 에러가 발생
- 해결 방법으로는
```
cap = cv2.VideoCapture(0, cv2.CAP_V4L)
```
- 변경하면 된다.
- 혹은 라즈베리 파이에 GStreamer 가 다운이 안되어 있을 수 있다.
```
https://qengineering.eu/install-gstreamer-1.18-on-raspberry-pi-4.html
```
- 위의 페이지에 접속하여 가이드라인 순서대로 다운로드를 진행 후 다시 시도하면 된다.


<br>

### :three: Run
- 파일을 실행하기 전 먼저 권한을 실행 가능하게 바꿔준다.
```
$ sudo chmod +x publisher.py
```

<br>

- catkin_make
```
$ cd ~/py_test
$ catkin_make
```

<br>

- rosrun을 하여 실행한다. 
```
$ rosrun test_pkg punlisher.py
```

---

<br><br>

## :computer: Desktop

<br>

### ⚙️ CUDA & cuDNN Install
#### 0. 이전 버전 삭제
- 이전에 설치되어있는 cuda를 삭제해야 한다.
```
$ sudo apt-get purge nvidia*
$ sudo apt-get autoremove
$ sudo apt-get autoclean
$ sudo rm -rf /usr/local/cuda*
```
<br>

#### 1. CUDA 설치
- nvidia-driver 재설치
```
$ sudo apt install nvidia-driver-460
$ sudo reboot
```

<br>

- https://developer.nvidia.com/cuda-toolkit-archive 접속하여 원하는 버전을 선택후 설치
```
$ wget https://developer.download.nvidia.com/compute/cuda/11.2.1/local_installers/cuda_11.2.1_460.32.03_linux.run
$ sudo sh cuda_11.2.1_460.32.03_linux.run
```

#### 2. 설치 후
- ./bashrc에 아래와 같이 기입한다.
```
export PATH=/usr/local/cuda-11.2/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```

<br>

#### 1. cuDNN 설치
- cuDNN은 그래픽 카드 가속을 도와주는 library이다. 따로 다운 바당서 경로 설정을 해야한다.
- https://developer.nvidia.com/cudnn 접속
- nvidia 로그인 후 자신의 CUDA와 호환되는 버전을 다운로드 한다.
- 필자는 8.1.1 을 다운받았다.

<br>

- 다운 받은 경로에서 압축해제
```
$ tar -xzvf cudnn-11.2-linux-x64v8.1.1.33.tgz
```

<br>

- 설치
```
$ sudo cp cuda/include/cudnn*.h /usr/local/cuda/include
$ sudo cp -P cuda/lib64/libcudnn* /usr/local/cuda/lib64
$ sudo chmod a+r /usr/local/cuda/include/cudnn*.h /usr/local/cuda/lib64/libcudnn*
```

<br>

- 설치확인
```
$ cat /usr/local/cuda/include/cudnn_version.h | grep CUDNN_MAJOR -A 2
```
![1](https://user-images.githubusercontent.com/94280596/221053281-c16e1f75-d00a-46b0-8edd-d400c3a388db.png)

<br><br>

```
$ ldconfig -N -v $(sed 's/:/ /' <<< $LD_LIBRARY_PATH) 2>/dev/null | grep libcudnn
```
![2](https://user-images.githubusercontent.com/94280596/221053287-93176561-f124-4a4b-b69f-6562c90d1f1c.png)

<br><br>

---

### 📌  darknet_ros 설치 후 실시간 이미지 구독할 subscriber 패키지 생성

<br>

### :one: darknet_ros install
```
$ mkdir darknet_ros
$ cd darknet_ros
$ mkdir src
$ catkin_make
```

#### git clone
```
$ cd darknet_ros/src
$ git clone https://github.com/leggedrobotics/darknet_ros.git
$ cd ..
$ catkin_make
```

<br>

### :pencil2: Customizing launch/yaml file

#### darknet_ros.launch
```
$ cd ~/darknet_ros/src/darknet_ros/darknet_ros/launch
$ gedit darknet_ros.launch
```

<br>

#### ✔️ 위에서 /camera/image 로 노드를 전송하기 때문에 아래와 같이 똑같은 이름으로 변경해준다.
```
<arg name="launch_prefix" default=""/>
<arg name="image" default="/camera/image"/>
```
---

<br>

#### ros.yaml
```
$ cd ~/darknet_ros/src/darknet_ros/darknet_ros/config
$ gedit ros.yaml
```
<br>

#### ✔️ 위에서 /camera/image 로 노드를 전송하기 때문에 아래와 같이 똑같은 이름으로 변경해준다.
```
subscribers:

  camera_reading:
    topic: /camera/image
    queue_size: 1

```
---

<br><br>

### :two: Create Package "subscriber"
```
$ cd py_test/src
$ catkin_create_pkg test_pkg rospy
$ cd ~/py_test/src/test_pkg
$ nano subscriber.py
```

<br>

### :notebook: Subscriber.py 
```
#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def image_callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    cv2.imshow("Image", cv_image)
    cv2.waitKey(1)

def image_subscriber():
    rospy.init_node('image_subscriber', anonymous=True)
    rospy.Subscriber("/camera/image", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    image_subscriber()
```

<br>

### :three: Run
- 파일을 실행하기 전 먼저 권한을 실행 가능하게 바꿔준다.
```
$ sudo chmod +x subscriber.py
```

<br>

- catkin_make
```
$ cd ~/py_test
$ catkin_make
```

<br>

- rosrun을 하여 실행한다. 
```
$ rosrun test_pkg subscriber.py
```
---


<br><br><br>


## :turtle: Run project

<br>

### 🖥️ Desktop
```
$ roscore
```

<br>

### 🍓 Raspberry Pi
```
$ cd py_test
$ source devel/setup.bash
$ rosrun test_pkg punlisher.py
```

<br>

### 🖥️ Desktop
```
$ cd py_test
$ source devel/setup.bash
$ rosrun test_pkg subscriber.py
```

```
$ cd darknet_ros
$ source devel/setup.bash
$ roslaunch darknet_ros darknet_ros.launch
```

---

<br><br>


## 🔳 실행 결과

https://user-images.githubusercontent.com/94280596/221054615-48c30686-e886-4449-8260-2e772df68f90.mp4






