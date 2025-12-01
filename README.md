# Building and Functioning of My Autonomous Vehicle Project

## 1. Introduction

This project centers on the development of a sophisticated $1/10^{\text{th}}$ scale autonomous vehicle platform designed for research and experimentation in self-driving technologies. Unlike simple remote-controlled cars, this vehicle serves as a mobile robotics platform capable of perception, planning, and control using advanced hardware and AI frameworks.

The vehicle operates on two primary software methodologies:

* **DonkeyCar Framework:** A high-level Python self-driving library focused on behavioral cloning (imitation learning) using Deep Learning (TensorFlow/PyTorch).

* **ROS 2 (Robot Operating System):** A set of software libraries and tools used for building more complex, modular robot applications, often utilized for SLAM (Simultaneous Localization and Mapping) and deterministic navigation.

## 2. Hardware Architecture

The vehicle is built upon a standard RC car chassis (typically $1/16$ or $1/10$ scale) modified to carry computing and sensing payloads.



### 2.1 The "Brain": Single Board Computer (SBC)

The core of the vehicle is a Single Board Computer running Ubuntu Linux.

* **NVIDIA Jetson Nano / Xavier NX:** Historically used for their GPU capabilities (CUDA cores) to accelerate Neural Network inference and computer vision tasks.

* **Raspberry Pi (4B / 5):** The platform also supports Raspberry Pi 5 coupled with AI HATs (like the Hailo AI accelerator) for high CPU performance and cost-effectiveness.

### 2.2 Actuation and Power

* **VESC (Vedder Electronic Speed Controller):** Unlike standard hobbyist ESCs, this project utilizes the VESC. This open-source ESC provides sophisticated field-oriented control (FOC) for the brushless DC motor. Crucially, it provides **odometry data** (speed $v$ and position $x$) back to the computer, allowing for precise dead-reckoning navigation.

* **Steering Servo:** A standard PWM-controlled servo motor handles the physical turning of the front wheels.

* **Power System:** The vehicle is powered by high-discharge Lithium Polymer (LiPo) batteries. A dedicated power distribution system ensures the SBC receives a steady $5\text{V}$ while the motor receives full battery voltage (e.g., $11.1\text{V}$ for a 3S LiPo).

* **Emergency Stop (EMO):** A mandatory safety circuit involving a wireless relay and visual indicators (Red/Blue LEDs). This hardware-level kill switch allows the vehicle's propulsion to be cut remotely in case of software failure.

### 2.3 Perception Sensors

* **OAK-D Lite / Pro:** A smart AI camera that performs depth sensing (stereo vision) and object detection on-board. This offloads processing from the main SBC.

* **Lidar (LD06):** A $360^{\circ}$ Laser Range Finder used for mapping the environment (SLAM) and obstacle avoidance.

* **IMU:** Inertial Measurement Units are often integrated or added for determining orientation ($\theta$) and acceleration ($a$).

## 3. The Software Stack

### 3.1 Operating System & Environment

The SBC runs a custom-configured version of **Ubuntu Linux**. To manage software dependencies and prevent version conflicts (a common issue with TensorFlow and OpenCV), the system relies heavily on **Docker containers** or Python **Virtual Environments (`venv`)**.

### 3.2 The DonkeyCar Framework

This is the "Hello World" of the platform. It treats self-driving as a supervised learning problem.

1. **Architecture:** It uses a modular "part" system. The Camera, VESC, and Controller are all software "parts" that update continuously within a control loop, typically running at a frequency of $f = 20\text{Hz}$.

2. **OpenCV:** Built from source with CUDA support (on Jetson) to handle image processing efficiently.

3. **Deep Learning:** Uses TensorFlow or PyTorch to run Convolutional Neural Networks (CNNs) that predict steering angles $\delta$ and throttle values $\tau$ based on input images.

## 4. How It Functions: The Workflow

### 4.1 Phase 1: Configuration and Calibration

Before the car can move autonomously, it must be calibrated to understand its physical limits.

* **`myconfig.py`:** This is the master configuration file. I define the camera type (e.g., `CAMERA_TYPE = "OAKD"`), drive train type (`DRIVE_TRAIN_TYPE = "VESC"`), and PID controller coefficients here.

* **Calibration:** Using the `donkey calibrate` command, I determine the exact PWM (Pulse Width Modulation) values corresponding to actuator limits:

  * Steering limits: $\delta_{\text{left}}$ and $\delta_{\text{right}}$.

  * Throttle limits: $\tau_{\text{max\_fwd}}$ and $\tau_{\text{max\_rev}}$.

  * Neutral: $\tau_{\text{stop}}$.

  * *Note:* Incorrect calibration can result in the car running into walls or not moving at all.

### 4.2 Phase 2: Data Collection (Human-in-the-Loop)

The "training" of the vehicle is done through **Behavioral Cloning**.

1. I drive the car manually using a game controller (Logitech F710 or PS4).

2. As the car drives, the computer records "Tubs" of data.

3. **A Tub Record** consists of a pair $(I_t, u_t)$ where:

   * $I_t$: The image seen by the camera at time $t$.

   * $u_t$: The control vector applied manually $(\delta, \tau)$.

4. The goal is to drive $N > 50$ laps perfectly. If I drive poorly, the AI will learn to drive poorly.

### 4.3 Phase 3: Training

The collected data (thousands of image-steering pairs) is offloaded to a more powerful computer (or GPU cluster) for training.

* **The Neural Network:** A Convolutional Neural Network (CNN) is trained. It learns a mapping function $f(I) \rightarrow (\hat{\delta}, \hat{\tau})$ by minimizing the error between its predicted steering angle $\hat{\delta}$ and the actual steering angle $\delta$.

* **Output:** A model file (e.g., `mypilot.h5`).

### 4.4 Phase 4: Autonomous Driving (Inference)

1. The trained model is transferred back to the robot.

2. I execute `python manage.py drive --model=mypilot.h5`.

3. **The Control Loop (**$20\text{Hz}$**):**

   * **Input:** Camera captures a frame $I_t$.

   * **Inference:** The model analyzes the frame and outputs a float value (e.g., $-0.5$ for a left turn).

   * **Actuation:** The SBC sends the corresponding command to the VESC and Servo.

   * **Repeat:** This cycle repeats every $50\text{ms}$.

## 5. Advanced Capabilities: Path Following

Beyond simple camera-based cloning, this project utilizes **PointOneNav** and **RTK-GPS** for precise localization.

* **Path Recording:** The car records a series of GPS waypoints $P = \{p_1, p_2, ..., p_n\}$.

* **Path Following:** Instead of reacting to an image, the car calculates its Cross-Track Error (CTE) relative to the recorded path and uses a **PID Controller** to minimize the error:
  $$
  u(t) = K_p e(t) + K_i \int e(t) dt + K_d \frac{de(t)}{dt}
  $$
  Where $e(t)$ is the lateral distance from the path.

## README / Reference Links

Below are the key documentation links and resources utilized in this project:

* **DonkeyCar Official Documentation:**

  * [Main Docs](http://docs.donkeycar.com)

  * [Setup Jetson Nano](https://docs.donkeycar.com/guide/robot_sbc/setup_jetson_nano/)

  * [Calibrate Steering/Throttle](https://docs.donkeycar.com/guide/calibrate/)

* **Hardware Resources:**

  * [VESC Project (Speed Controller)](https://vesc-project.com/)

  * [NVIDIA Jetson GPIO Pinout](https://www.jetsonhacks.com/nvidia-jetson-nano-j41-header-pinout/)

  * [LD06 Lidar Datasheet](https://www.google.com/search?q=LD06+Lidar+Datasheet) (General reference)

* **Software & Libraries:**

  * [Jetson GPIO Python Library](https://github.com/NVIDIA/jetson-gpio)

  * [Luxonis DepthAI (OAK-D Camera)](https://github.com/luxonis/depthai-python)

  * [TensorFlow Install Guide for Jetson](https://docs.nvidia.com/deeplearning/frameworks/install-tf-jetson-platform/index.html)

* **Tools:**

  * [Fritzing (Circuit Diagramming)](http://fritzing.org/home/)

  * [Etcher (SD Card Imaging)](https://etcher.io/)
