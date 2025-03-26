#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

import pyaudio
import wave
import socket
import time


class AudioClient(Node): 

    def __init__(self):
        super().__init__('audio_client')
        self.declare_parameter('frequency', 11)
        self.frequency = self.get_parameter('frequency').value
        self.timer = 1.0 / self.frequency
        self.timer_input = self.create_timer(self.timer, self.input_audio)
        self.timer_output = self.create_timer(self.timer, self.output_audio)

        self.subscription = self.create_subscription(
            Bool,
            'audio_status',
            self.status_audio,
            10)
        self.subscription  # prevent unused variable warning

        # Configuration of connection
        self.HOST = '192.168.1.100'
        self.PORT = 8888
        self.client_conected =  False
        self.CHUNK_CONNECTION = 4096

        # Configuration of audio
        self.CHUNK = 1024  # Number of audio samples per frame
        self.FORMAT = pyaudio.paInt16  # Audio format
        self.CHANNELS = 2  # Number of audio channels
        self.RATE = 44100  # Sampling rate (samples per second)
        self.alive = False

        audio = pyaudio.PyAudio()

        # Open the audio stream for input (microphone)
        self.stream_input = audio.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            input=True,
            frames_per_buffer=self.CHUNK
        )

        # Open the audio stream for output (speaker)
        self.stream_output = audio.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            output=True,
            frames_per_buffer=self.CHUNK
        )

        self.output_filename = 'audio-recording.wav'

        # Create a TCP socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_conected =  False

        self.start()

    def status_audio(self, msg):
        self.alive = msg.data
        if self.alive:
            self.set_parameters([Parameter('frequency', Parameter.Type.DOUBLE, 11)])
            self.frequency = 11
        else:
            self.set_parameters([Parameter('frequency', Parameter.Type.DOUBLE, 0.1)])
            self.frequency = 0.1
        self.timer_input.reset()
        self.timer_output.reset()

    def input_audio(self):
        if self.client_conected and self.alive:
            try:
                wav_file_wb = wave.open(self.output_filename, "wb")
                wav_file_wb.setnchannels(2)
                wav_file_wb.setsampwidth(2)
                wav_file_wb.setframerate(48000)
                # recording audio and saved
                input_audio = self.stream_input.read(5120, exception_on_overflow=False)
                wav_file_wb.writeframes(input_audio)
                # encode and send audio
                wav_file_rb = wave.open(self.output_filename, "rb")
                data = wav_file_rb.readframes(self.CHUNK)
                recv = self.server_socket.recv(self.CHUNK_CONNECTION)
                if recv == "Received":
                    self.server_socket.send(data, (self.HOST, self.PORT))
                else:
                    self.get_logger().warn("Error back message: Don't received")
            except Exception as e:
                self.get_logger().error(f"Error in play_audio: {e}")
            except ConnectionAbortedError:
                self.get_logger().error("Conection aborted ")
            except ConnectionResetError:
                self.client_conected = False
                self.server_socket.close()
                self.get_logger().warning("The client was desconected ")
                self.try_connect()            

    def output_audio(self):
        if self.client_conected and self.alive:
            try:
                # Receive audio data from the other device
                data = self.server_socket.recv(self.CHUNK_CONNECTION)
                # Play the received audio data
                self.stream_output.write(data, self.CHUNK)
                self.server_socket.send("Received")

            except Exception as e:
                self.get_logger().error(f"Error in receive_audio: {e}")
            except ConnectionResetError:
                self.client_conected = False
                self.server_socket.close()
                self.get_logger().warning("The client was desconected ")
                self.try_connect()


    def try_connect(self):
            try:
                self.get_logger.info("Trying connect to server")
                self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.server_socket.connect((self.HOST, self.PORT))
                self.get_logger.info(f"Conection established with the server {self.HOST}") 
                self.client_conected = True

            except ConnectionRefusedError:
                self.get_logger.warn("Conection refused ")
                self.conected = False
                self.server_socket.close()
                time.sleep(3)

            except  ConnectionAbortedError:
                self.get_logger.warn("Conection aborted ")
                self.conected = False
                self.server_socket.close()
                time.sleep(3)

    def start(self):
        try:
            self.try_connect()
            self.client_connect.send("Received")

        except TypeError:
            self.get_logger.error("Type error")

        except ConnectionResetError:
            self.client_conected = False
            self.server_socket.close()
            self.get_logger().warn("The client was desconected ")

    def destroy_node(self):
        self.stream_input.stop_stream()
        self.stream_input.close()
        self.stream_output.stop_stream()
        self.stream_output.close()
        self.audio.terminate()
        if self.server_socket:
            self.server_socket.close()
        self.server_socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    audio_client = AudioClient()

    rclpy.spin(AudioClient)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    audio_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()