# NAOqi Speech for ROS 2

This ROS 2 package provides a bridge to the speech and audio functionalities of SoftBank Robotics' Pepper and NAO robots, which run on the NAOqi OS. It exposes NAOqi's `ALTextToSpeech`, `ALSpeechRecognition`, `ALAudioPlayer`, and `ALAudioDevice` modules as ROS 2 services and topics, allowing for easy integration into a ROS 2 ecosystem.

## Features

*   **Text-to-Speech (TTS)**: Make the robot speak using animated or regular speech.
*   **Speech Recognition**: Configure the robot's vocabulary and receive recognized words.
*   **Audio Playback**: Play sound files from the robot or audio from a web stream.
*   **Volume Control**: Get and set the robot's master volume.
*   **Asynchronous Operations**: Long-running actions like speaking are handled in a non-blocking way, allowing the node to remain responsive.

## Dependencies

*   `rclpy`
*   `std_srvs`
*   `naoqi_utilities_msgs`: Contains the custom message and service definitions used by this node.

## How to Run the Node

To start the node, you need to provide the IP address and port of the robot.

```bash
ros2 run naoqi_speech naoqi_speech_node --ros-args -p ip:=<robot_ip> -p port:=<robot_port>
```

For example:
```bash
ros2 run naoqi_speech naoqi_speech_node --ros-args -p ip:=192.168.1.101 -p port:=9559
```

## ROS 2 API

All services and topics are exposed under the node's namespace (`/naoqi_speech_node/` by default).

### Services

*   **`~/say`** ([naoqi_utilities_msgs/srv/Say](naoqi_utilities_msgs/srv/Say.srv))  
    Makes the robot say a given text. Can be animated, and run synchronously or asynchronously.

*   **`~/shut_up`** ([std_srvs/srv/Trigger](https://docs.ros2.org/foxy/api/std_srvs/srv/Trigger.html))  
    Stops the robot from speaking immediately.

*   **`~/configure_speech`** ([naoqi_utilities_msgs/srv/ConfigureSpeech](naoqi_utilities_msgs/srv/ConfigureSpeech.srv))  
    Configures the speech recognition engine. You can set the vocabulary, language, and enable/disable audio-visual expressions during recognition.

*   **`~/set_volume`** ([naoqi_utilities_msgs/srv/SetVolume](naoqi_utilities_msgs/srv/SetVolume.srv))  
    Sets the robot's output volume (0-100).

*   **`~/get_volume`** ([naoqi_utilities_msgs/srv/GetVolume](naoqi_utilities_msgs/srv/GetVolume.srv))  
    Gets the current robot's output volume.

*   **`~/play_sound`** ([naoqi_utilities_msgs/srv/PlaySound](naoqi_utilities_msgs/srv/PlaySound.srv))  
    Plays a sound file located on the robot's file system.

*   **`~/play_web_stream`** ([naoqi_utilities_msgs/srv/PlayWebStream](naoqi_utilities_msgs/srv/PlayWebStream.srv))  
    Plays an audio stream from a given URL.

*   **`~/stop_all_sounds`** ([std_srvs/srv/Trigger](https://docs.ros2.org/foxy/api/std_srvs/srv/Trigger.html))  
    Stops all sounds being played by the audio player.

### Published Topics

*   **`~/word_recognized`** ([naoqi_utilities_msgs/msg/WordConfidence](naoqi_utilities_msgs/msg/WordConfidence.msg))  
    Publishes a word and its confidence score when the robot recognizes speech.

## Usage Example

To make the robot say "Hello world" in English with animation:

```bash
ros2 service call /naoqi_speech_node/say naoqi_utilities_msgs/srv/Say "{text: 'Hello world', language: 'English', animated: true}"
```

To set a vocabulary for speech recognition:
```bash
ros2 service call /naoqi_speech_node/configure_speech naoqi_utilities_msgs/srv/ConfigureSpeech "{vocabulary: ['yes', 'no', 'hello'], update_recognition_settings: true, recognition_enabled: true}"
```
Then, listen to the topic to get recognized words:
```bash
ros2 topic echo /naoqi_speech/word_recognized
```