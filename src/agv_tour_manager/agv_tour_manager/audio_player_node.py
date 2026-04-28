#!/usr/bin/env python3
"""Audio Player Node - plays audio commentary at tour stops."""
import os
import subprocess
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class AudioPlayerNode(Node):
    def __init__(self):
        super().__init__('audio_player_node')
        self.declare_parameter('audio_dir', '/opt/agv/audio')
        self.declare_parameter('volume', 80)
        self.declare_parameter('player_cmd', 'aplay')

        self.audio_dir = self.get_parameter('audio_dir').value
        self.volume = self.get_parameter('volume').value
        self.player_cmd = self.get_parameter('player_cmd').value

        self.current_proc = None
        self.is_playing = False

        self.play_sub = self.create_subscription(String, '/agv/play_audio', self.play_callback, 10)
        self.stop_sub = self.create_subscription(Bool, '/agv/stop_audio', self.stop_callback, 10)
        self.status_pub = self.create_publisher(Bool, '/agv/audio_playing', 10)
        self.create_timer(1.0, self.status_check)
        self.get_logger().info(f'Audio Player Node initialized (dir: {self.audio_dir})')

    def play_callback(self, msg: String):
        filename = msg.data
        if not filename:
            return
        filepath = os.path.join(self.audio_dir, filename) if not os.path.isabs(filename) else filename
        if not os.path.exists(filepath):
            self.get_logger().warn(f'Audio file not found: {filepath}')
            return
        self._stop_current()
        self.get_logger().info(f'Playing: {filepath}')
        def play():
            try:
                self.is_playing = True
                self.current_proc = subprocess.Popen(
                    [self.player_cmd, filepath],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                self.current_proc.wait()
            except Exception as e:
                self.get_logger().error(f'Audio playback error: {e}')
            finally:
                self.is_playing = False
                self.current_proc = None
        t = threading.Thread(target=play, daemon=True)
        t.start()

    def stop_callback(self, msg: Bool):
        if msg.data:
            self._stop_current()

    def _stop_current(self):
        if self.current_proc and self.current_proc.poll() is None:
            self.current_proc.terminate()
            self.current_proc = None
        self.is_playing = False

    def status_check(self):
        msg = Bool()
        msg.data = self.is_playing
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AudioPlayerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
