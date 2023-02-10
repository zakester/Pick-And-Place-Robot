
# Mobile Pick And Place Robot with Computer Vision

The project was created for my Bachelor Degree in Automation, our focus was to make the part were the robot can detect the target object and try to go to it's location (there is no pick or place in this part)

This project consist of a mobile robot that follows the direction of the targeted container, it rotates to the point where it gets perpendicular with the detected object and then proceeds to march ahead until it reaches a certain distance away from the object, all this using various sensors, motors, Computer Vision, and mathematical methods.

## Approach
Computer Vision part will be in a smartphone then we will send (x, y, width, label, rotationDegree) to ESP8266 through WiFi.

ESP8266 will calculate how much the robot will need to rotate, then the rotation will start which is controlled with PID Controller. While that the ESP8266 will send the current state to the android app and generate graphs.

After rotation process is done Ultrasonic will check if distance is more then 30cm so the robot will move forward until the condition is no longer true.

## Hardware
- NodeMCU ESP8266.
- L298N.
- 3V DC motor.
- Optical Encoder.
- Ultrasonic Sensor.
- Smart Phone (I used Redmi Note 8).


## Screenshots
![01](/assets/images/01.png)
![02](/assets/images/02.png)
![03](/assets/images/03.png)

## Video
![01](/assets/gif/01.gif)

## Circuit
![Circuit](/assets/images/circuit.png)

## PID
![PID](/assets/images/pid.png)
# Flowchart
![Flowchart](/assets/images/flowchart.png)
## Dataset
About 720 images for Target Package, Normal Package and Zone using smartphone Xiaomi Redmi Note 8 each image has 448x448px resolution.

![01](/assets/images/01.png)
![02](/assets/images/02.png)
![03](/assets/images/03.png)

The annotation files are xml files that contain some necessary information like path, width, height, label, x, y…ect, each image will be fed into the model with it’s annotation file, The annotation is done by LabelImg software.

We choose our labels to be as follow:
- Package		-> package
- Target Package 	-> package_target
- Zone 			-> zone

- [__Kaggle Link to Dataset__](https://www.kaggle.com/datasets/hamzahadjammar/pick-and-place-packages-mobile-robot)
# Computer Vision
In order to distinguish the __*Target Package*__ from the __*Normal Packages*__, and in order to recognize the __*Zone*__, we train an Object Detection Model using Tensorflow Library.

This object detection model is going to give us __x, y__ coordinates of each detected object and from there the robot will turn and move according to this information.

Dataset trainded on __EfficientDet-Lite2__ model. Not the best, we can try find better models

| # | Model Architecture | Size(MB) | Latency(ms) | Average Precision |
|:--------:|:----------:|:----------:|:----------:|:----------:|
| 1 | EfficientDet-Lite2 | 7.2 | 396 | 33.97% |



# AI Training Results
| # | batch_size | epochs | AP - .tf | AP - .tflite |
|:--------:|:----------:|:----------:|:----------:|:----------:|
| 1 | 4 | 20 | 0.78 | 0.76 |
| 2 | 16 | 20 | 0.81 | 0.79 |
| 3 | 32 | 20 | 0.79 | 0.77 |
| 4 | 32 | 50 | 0.83 | 0.81 |
| 5 | 32 | 60 | 0.81 | 0.78 |

- [__You can check the code for training using here__](notebook/pick-and-place-32-batch-size-60-epochs.ipynb)
# Libraries
__C++__
- ESP8266WebServer
- ESP8266WiFi
- ArduinoJson
__Python__
- tensorflow 2.5.0
- tflite-model-maker --user
- pycocotools
- tflite-support
