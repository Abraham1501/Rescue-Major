import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import pyaudio
import socket


class video_client(Node):

    def __init__(self):
        super().__init__('video_client')
        self.publisher_ = self.create_publisher(String, 'audio_input', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription = self.create_subscription(
            String,
            'audio_output',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Configuration of connection
        HOST = '192.168.1.100'
        PORT = 8888

        #Configuration of audio
        CHUNK = 1024  # Number of audio samples per frame
        FORMAT = pyaudio.paInt16  # Audio format
        CHANNELS = 2  # Number of audio channels
        RATE = 44100  # Sampling rate (samples per second)

        audio = pyaudio.PyAudio()

        # Open the audio stream for input (microphone)
        stream_input = audio.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            frames_per_buffer=CHUNK
        )

        # Open the audio stream for output (speaker)
        stream_output = audio.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            frames_per_buffer=CHUNK
        )



    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    video_client = video_client()

    rclpy.spin(video_client)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    video_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()