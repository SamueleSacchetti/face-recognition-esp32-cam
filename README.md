# Face recognition using ESP32-Cam 
ESP32-Cam is an edge device with low memory capacity and low computational performance. It can be used for many tasks such as automation, security and surveillance, and so on. 
In this job, we used the ESP32-Cam to perform real-time inference using a pre-trained face recognition model, handling the output with LEDs based on the result obtained.

## Failed steps before the success
  ### First Approach
We started with a "manual approach", performing a fine-tuning process on the MobileNetV2 model. The dataset used was composed of our faces and unknown people faces pictures. During the implementation (using Micropython as programming language and Thonny as IDE), we faced two main problems:
    1. the model we got was too big to save and run on the device;
    2. there is not a Tensorflow Lite library for Micropython.
    
This solution can be found [here](./python_customModel).

  ### Second Approach
After the experience with Micropython and Tensorflow Lite, we converted the .savedModel file into a byte array C so we could upload the model on the device in an easier way. After that the second problem we had in the first approach has been solved. However the first problem remained cause of the model's size.
So we found a "person detection" model provided by Tensorflow Lite. The model's output is a confidence score that says if a person is in front of the camera or not. In this second approach we used Arduino as IDE but the results obtained were not satistfying cause of the low accuracy of the results.

This second solution can be found [here](./tflite_arduino_person_detection).
  

  ### Final Approach: the success!

## How we achieved the result

## Inference of the face recognition model

## How to replicate the obtained results
