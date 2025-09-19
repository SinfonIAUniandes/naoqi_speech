import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import qi
import argparse
import sys
import numpy as np

from std_srvs.srv import Trigger
from naoqi_utilities_msgs.msg import WordConfidence
from naoqi_bridge_msgs.msg import AudioBuffer, Bumper, HandTouch, HeadTouch
from naoqi_utilities_msgs.srv import (
    ConfigureSpeech, SetVolume, GetVolume, PlaySound, PlayWebStream, Say
)

class NaoqiSpeechNode(Node):
    """
    ROS2 Node to manage speech and audio functionalities of a NAO robot.
    """
    def __init__(self, session):
        """
        Initializes the node, NAOqi service clients, ROS2 services, and publishers.
        """
        super().__init__('naoqi_speech_node')
        self.get_logger().info("Initializing NaoqiSpeechNode...")
        self.language = "English" # Default language

        # Create a reentrant callback group to allow parallel execution of long-running services
        self.reentrant_group = ReentrantCallbackGroup()

        # --- NAOqi Service Clients ---
        try:
            self.al_audio_device = session.service("ALAudioDevice")
            self.al_speech_recognition = session.service("ALSpeechRecognition")
            self.al_audio_player = session.service("ALAudioPlayer")
            self.al_text_to_speech = session.service("ALTextToSpeech")
            self.al_speaking_movement = session.service("ALSpeakingMovement")
            self.al_animated_speech = session.service("ALAnimatedSpeech")
            self.al_memory = session.service("ALMemory")
            self.get_logger().info("NAOqi service clients obtained successfully.")
        except Exception as e:
            self.get_logger().error(f"Could not connect to NAOqi services: {e}")
            sys.exit(1)

        # --- ROS2 Publishers ---
        self.word_recognized_pub = self.create_publisher(
            WordConfidence,
            '~/word_recognized',
            10
        )
        self.mic_publisher = self.create_publisher(AudioBuffer, '/mic', 10)
        self.bumper_pub = self.create_publisher(Bumper, '/bumper', 10)
        self.hand_touch_pub = self.create_publisher(HandTouch, '/hand_touch', 10)
        self.head_touch_pub = self.create_publisher(HeadTouch, '/head_touch', 10)

        # --- ROS2 Services ---
        # ALTextToSpeech & ALSpeakingMovement
        # Assign potentially blocking services to the reentrant group
        self.say_service = self.create_service(
            Say, '~/say', self.say_callback, callback_group=self.reentrant_group
        )
        self.shut_up_service = self.create_service(
            Trigger, '~/shut_up', self.shut_up_callback, callback_group=self.reentrant_group
        )

        # ALAudioDevice
        self.set_volume_service = self.create_service(SetVolume, '~/set_volume', self.set_volume_callback)
        self.get_volume_service = self.create_service(GetVolume, '~/get_volume', self.get_volume_callback)

        # ALSpeechRecognition
        self.configure_speech_service = self.create_service(
            ConfigureSpeech, '~/configure_speech', self.configure_speech_callback
        )

        # ALAudioPlayer
        self.play_sound_service = self.create_service(PlaySound, '~/play_sound', self.play_sound_callback)
        self.play_web_stream_service = self.create_service(PlayWebStream, '~/play_web_stream', self.play_web_stream_callback)
        self.stop_all_sounds_service = self.create_service(Trigger, '~/stop_all_sounds', self.stop_all_sounds_callback)

        # --- NAOqi Event Subscribers ---
        # Using subscribeToMicroEvent for compatibility with NAOqi 2.5
        # The node must be registered as a NAOqi service to receive the callback.
        # This is done in _register_audio_service.
        self.al_memory.subscribeToMicroEvent(
            "WordRecognized",
            "NaoqiSpeechNode_Audio",
            "WordRecognized",
            "on_word_recognized"
        )

        # Subscribe to touch events
        self._subscribe_touch_events()

        # --- NAOqi Audio Service for Microphone ---
        self.naoqi_audio_service_name = "NaoqiSpeechNode_Audio"
        self.mic_sample_rate = 16000
        self._register_audio_service(session)


        self.get_logger().info("Speech functionalities node is ready.")

    def _register_audio_service(self, session):
        """
        Registers a NAOqi service to get the audio stream from the microphones.
        """
        try:
            session.registerService(self.naoqi_audio_service_name, self)
            # Set client preferences: 16000 Hz, 4 channels (ALL), deinterleaved
            self.al_audio_device.setClientPreferences(self.naoqi_audio_service_name, self.mic_sample_rate, 3, 0)
            self.al_audio_device.subscribe(self.naoqi_audio_service_name)
            self.get_logger().info(f"Successfully subscribed to ALAudioDevice with service name '{self.naoqi_audio_service_name}'.")
        except Exception as e:
            self.get_logger().error(f"Failed to register audio service with NAOqi: {e}")

    def processRemote(self, nbOfChannels, nbOfSamples, timeStamp, buffer):
        """
        This method is called by NAOqi's ALAudioDevice when new audio data is available.
        It processes the raw audio buffer and publishes it to a ROS topic.
        """
        try:
            msg = AudioBuffer()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.frequency = self.mic_sample_rate
            # The channel map can be specified if needed, but is left empty for now.
            # msg.channel_map = [msg.CHANNEL_FRONT_LEFT, msg.CHANNEL_FRONT_CENTER, msg.CHANNEL_FRONT_RIGHT, msg.CHANNEL_REAR_CENTER]
            
            # Convert the raw buffer to a NumPy array of 16-bit integers
            audio_data = np.frombuffer(buffer, dtype=np.int16)
            msg.data = audio_data.tolist()
            
            self.mic_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to process and publish audio buffer: {e}")


    # --- Event Callbacks ---
    def on_word_recognized(self, key, value, message):
        """
        NAOqi event callback for when a word is recognized.
        Publishes the word and its confidence level.
        Signature is for subscribeToMicroEvent: (key, value, message)
        """
        if value and len(value) == 2:
            word, confidence = value
            self.get_logger().info(f"Word Recognized: '{word}' with confidence {confidence:.2f}")
            msg = WordConfidence()
            msg.word = word
            msg.confidence = confidence
            self.word_recognized_pub.publish(msg)

    def on_head_touch(self, key, value, message):
        """NAOqi event callback for head touch sensors."""
        msg = HeadTouch()
        if "Front" in key:
            msg.button = HeadTouch.BUTTON_FRONT
        elif "Middle" in key:
            msg.button = HeadTouch.BUTTON_MIDDLE
        elif "Rear" in key:
            msg.button = HeadTouch.BUTTON_REAR
        else:
            return  # Unknown button

        msg.state = HeadTouch.STATE_PRESSED if value > 0.5 else HeadTouch.STATE_RELEASED
        self.get_logger().info(f"Head touch event: button={msg.button}, state={msg.state}")
        self.head_touch_pub.publish(msg)

    def on_hand_touch(self, key, value, message):
        """NAOqi event callback for hand touch sensors."""
        msg = HandTouch()
        if "HandRightBack" in key:
            msg.hand = HandTouch.RIGHT_BACK
        elif "HandRightLeft" in key:
            msg.hand = HandTouch.RIGHT_LEFT
        elif "HandRightRight" in key:
            msg.hand = HandTouch.RIGHT_RIGHT
        elif "HandLeftBack" in key:
            msg.hand = HandTouch.LEFT_BACK
        elif "HandLeftLeft" in key:
            msg.hand = HandTouch.LEFT_LEFT
        elif "HandLeftRight" in key:
            msg.hand = HandTouch.LEFT_RIGHT
        else:
            return # Unknown hand sensor

        msg.state = HandTouch.STATE_PRESSED if value > 0.5 else HandTouch.STATE_RELEASED
        self.get_logger().info(f"Hand touch event: hand={msg.hand}, state={msg.state}")
        self.hand_touch_pub.publish(msg)

    def on_bumper_touch(self, key, value, message):
        """NAOqi event callback for bumper/feet touch sensors."""
        msg = Bumper()
        if "RightBumper" in key:
            msg.bumper = Bumper.RIGHT
        elif "LeftBumper" in key:
            msg.bumper = Bumper.LEFT
        elif "BackBumper" in key:
            msg.bumper = Bumper.BACK
        else:
            return # Unknown bumper

        msg.state = Bumper.STATE_PRESSED if value > 0.5 else Bumper.STATE_RELEASED
        self.get_logger().info(f"Bumper event: bumper={msg.bumper}, state={msg.state}")
        self.bumper_pub.publish(msg)

    def _subscribe_touch_events(self):
        """Subscribes to all touch-related NAOqi events."""
        touch_events = {
            # Head
            "FrontTactilTouched": "on_head_touch",
            "MiddleTactilTouched": "on_head_touch",
            "RearTactilTouched": "on_head_touch",
            # Hands
            "HandRightBackTouched": "on_hand_touch",
            "HandRightLeftTouched": "on_hand_touch",
            "HandRightRightTouched": "on_hand_touch",
            "HandLeftBackTouched": "on_hand_touch",
            "HandLeftLeftTouched": "on_hand_touch",
            "HandLeftRightTouched": "on_hand_touch",
            # Bumpers
            "RightBumperPressed": "on_bumper_touch",
            "LeftBumperPressed": "on_bumper_touch",
            "BackBumperPressed": "on_bumper_touch", # Pepper specific
        }
        for event, callback_method in touch_events.items():
            try:
                self.al_memory.subscribeToMicroEvent(
                    event,
                    "NaoqiSpeechNode_Audio",
                    event, # Message for disambiguation
                    callback_method
                )
            except Exception as e:
                self.get_logger().warn(f"Could not subscribe to event '{event}': {e}")

    # --- Service Callbacks ---
    def say_callback(self, request, response):
        """
        Makes the robot say a text with multiple options.
        This callback can block, but won't freeze the node thanks to the ReentrantCallbackGroup.
        """
        try:
            self.get_logger().info(f"Request to say: '{request.text}' [Lang: {request.language}, Animated: {request.animated}, Async: {request.asynchronous}]")

            # Set language if it's different from the current one
            if request.language and request.language != self.language:
                self.language = request.language
                self.al_text_to_speech.setLanguage(self.language)
                self.get_logger().info(f"Language changed to {self.language}")

            speech_service = self.al_animated_speech if request.animated else self.al_text_to_speech
            self.al_speaking_movement.setEnabled(request.animated)

            if request.asynchronous:
                speech_service.say(request.text, _async=True)
                response.message = "Asynchronous say command sent."
            else:
                # This is a blocking call. The ReentrantCallbackGroup and MultiThreadedExecutor
                # ensure this only blocks the current thread, not the entire node.
                speech_service.say(request.text)
                response.message = "Synchronous say command finished."

            response.success = True
        except Exception as e:
            response.success = False
            response.message = f"Error in say service: {e}"
            self.get_logger().error(response.message)
            
        return response

    def shut_up_callback(self, request, response):
        try:
            self.get_logger().info("Request to stop all speech.")
            self.al_text_to_speech.stopAll()
            response.success = True
            response.message = "All speech stopped."
        except Exception as e:
            response.success = False
            response.message = f"Error in shut_up service: {e}"
            self.get_logger().error(response.message)
        return response

    def set_volume_callback(self, request, response):
        try:
            self.get_logger().info(f"Request to set volume to {request.volume}.")
            self.al_audio_device.setOutputVolume(request.volume)
            response.success = True
            response.message = "Volume set."
        except Exception as e:
            response.success = False
            response.message = f"Error setting volume: {e}"
            self.get_logger().error(response.message)
        return response

    def get_volume_callback(self, request, response):
        try:
            volume = self.al_audio_device.getOutputVolume()
            self.get_logger().info(f"Current volume is {volume}.")
            response.volume = volume
        except Exception as e:
            self.get_logger().error(f"Error getting volume: {e}")
        return response

    def configure_speech_callback(self, request, response):
        """
        Unified callback to configure speech recognition settings.
        """
        try:
            self.get_logger().info("Request to configure speech.")
            
            # Pause recognition to apply changes safely
            self.al_speech_recognition.pause(True)
            
            # 1. Set Vocabulary
            if request.vocabulary:
                vocabulary = list(request.vocabulary)
                self.get_logger().info(f"Setting vocabulary to: {vocabulary}")
                self.al_speech_recognition.setVocabulary(vocabulary, False)

            # 2. Set Language
            if request.language:
                lang = request.language
                self.get_logger().info(f"Setting language to: {lang}")
                self.al_speech_recognition.setLanguage(lang)
                self.al_text_to_speech.setLanguage(lang)
                self.language = lang # Update node's current language

            # 3. Configure Recognition (Enabled, Audio/Visual Expressions)
            if request.update_recognition_settings:
                self.get_logger().info(f"Configuring recognition: enabled={request.recognition_enabled}, "
                                       f"audio_expr={request.audio_expression_enabled}, visual_expr={request.visual_expression_enabled}")
                if request.recognition_enabled:
                    self.al_speech_recognition.subscribe("NaoqiSpeechNode_Recognition")
                else:
                    self.al_speech_recognition.unsubscribe("NaoqiSpeechNode_Recognition")
                
                self.al_speech_recognition.setAudioExpression(request.audio_expression_enabled)
                self.al_speech_recognition.setVisualExpression(request.visual_expression_enabled)

            # Resume recognition after applying all changes
            self.al_speech_recognition.pause(False)
            
            response.success = True
            response.message = "Speech configuration updated successfully."

        except Exception as e:
            # In case of error, try to resume recognition anyway
            try:
                self.al_speech_recognition.pause(False)
            except Exception as resume_e:
                self.get_logger().error(f"Failed to resume speech recognition after another error: {resume_e}")
            
            response.success = False
            response.message = f"Error configuring speech: {e}"
            self.get_logger().error(response.message)
            
        return response

    def play_sound_callback(self, request, response):
        """
        Plays a single audio file located on the robot's file system.
        """
        try:
            self.get_logger().info(f"Request to play sound file: {request.file_path}")
            self.al_audio_player.playFile(request.file_path)
            response.success = True
            response.message = "Play sound command sent."
        except Exception as e:
            response.success = False
            response.message = f"Error playing sound file: {e}"
            self.get_logger().error(response.message)
        return response

    def play_web_stream_callback(self, request, response):
        """
        Plays an audio stream from a URL asynchronously, returning immediately.
        """
        try:
            self.get_logger().info(f"Request to play web stream asynchronously: {request.url}")
            # Call with _async=True to make it non-blocking
            self.al_audio_player.playWebStream(request.url, 1.0, 0.0, _async=True)
            response.success = True
            response.message = "Play web stream command sent asynchronously."
        except Exception as e:
            response.success = False
            response.message = f"Error playing web stream: {e}"
            self.get_logger().error(response.message)
        return response

    def stop_all_sounds_callback(self, request, response):
        try:
            self.get_logger().info("Request to stop all audio player sounds.")
            self.al_audio_player.stopAll()
            response.success = True
            response.message = "All audio player sounds stopped."
        except Exception as e:
            response.success = False
            response.message = f"Error stopping sounds: {e}"
            self.get_logger().error(response.message)
        return response

    def on_shutdown(self):
        """
        Called when the node is shutting down. Unsubscribes the audio service.
        """
        try:
            self.al_audio_device.unsubscribe(self.naoqi_audio_service_name)
            self.get_logger().info("Audio module unsubscribed successfully.")
        except Exception as e:
            self.get_logger().warn(f"Error while unsubscribing audio module: {e}")

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot IP address. On Robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")
    parsed_args, _ = parser.parse_known_args(args=sys.argv[1:])

    session = qi.Session()
    try:
        session.listen("tcp://0.0.0.0:0")
    except RuntimeError as e:
        rclpy.logging.get_logger('naoqi_speech_node').error(f"Failed to set listen URL: {e}")
    
    
    try:
        session.connect(f"tcp://{parsed_args.ip}:{parsed_args.port}")
    except RuntimeError:
        print(f"Can't connect to Naoqi at ip \"{parsed_args.ip}\" on port {parsed_args.port}.\n"
              "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    naoqi_speech_node = NaoqiSpeechNode(session)
    
    # Use a MultiThreadedExecutor to handle callbacks in parallel.
    # num_threads can be adjusted based on expected concurrent calls.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(naoqi_speech_node)

    try:
        # Spin the executor to process callbacks
        executor.spin()
    except KeyboardInterrupt:
        print("Closing the speech functionalities node.")
    finally:
        # Custom shutdown logic
        naoqi_speech_node.on_shutdown()
        # Shutdown the executor and destroy the node
        executor.shutdown()
        naoqi_speech_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()