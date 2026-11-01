#!/usr/bin/env python3
"""标定用机械臂手动点动 (按一下就到位, 不连续运动)"""

import math, time, signal, sys, os, select, termios, tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from tl_ros2_interface.srv import OpenServoJ


class JogNode(Node):
    def __init__(self):
        super().__init__('jog')
        self.latest_joint = None
        self.create_subscription(JointState, '/joint_states', self.jcb, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/tl_driver/set_servoj_pos', 10)
        self.srv = self.create_client(OpenServoJ, '/tl_driver/open_servoj')
        self.srv.wait_for_service(5)
        req = OpenServoJ.Request()
        req.vmax = [10.]*7; req.amax = [3000.]*7; req.jmax = [50000.]*7
        self.srv.call_async(req)
        # 等关节数据
        t0=time.time()
        while self.latest_joint is None and time.time()-t0<5:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_joint:
            self.pos = [v*180./math.pi for v in self.latest_joint.position]
            self.pos = (self.pos+[0.]*7)[:7]
        else:
            self.pos = [0.]*7
        self.create_timer(0.01, self.timer_cb)
        # 键盘
        try:
            self.fd = sys.stdin.fileno() if sys.stdin.isatty() else os.open('/dev/tty', os.O_RDONLY|os.O_NONBLOCK)
            self.old = termios.tcgetattr(self.fd); tty.setcbreak(self.fd)
        except: self.fd=None
        print("=== 手动点动 ===")
        print("j1-/j1+  j2-/j2+  ...  z回零  q退出")
        print("每次按移动10度")

    def jcb(self, m): self.latest_joint = m

    def timer_cb(self):
        m = Float64MultiArray()
        m.data = [float(v) for v in self.pos]
        self.pub.publish(m)

    def read_key(self):
        if not self.fd: return None
        try:
            dr,_,_ = select.select([self.fd],[],[],0)
            return os.read(self.fd,1).decode(errors='ignore') if dr else None
        except: return None

    def apply(self, keys):
        if 'q' in keys: return False
        inc = 10.0
        mapping = {'1':0,'2':1,'3':2,'4':3,'5':4,'6':5,'!':0,'@':1,'#':2,'$':3,'%':4,'^':5}
        # shift+数字 = 负方向
        shift_map = {'!':0,'@':1,'#':2,'$':3,'%':4,'^':5}
        for k in keys:
            if k == 'z':
                self.pos = [0.]*7; print("回零")
            elif k in mapping:
                j = mapping[k]
                d = -inc if k in shift_map else inc
                self.pos[j] += d
                print(f"J{j+1}: {self.pos[j]:.1f}")
        return True

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            k = self.read_key()
            if k and not self.apply(k): break
        try: termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
        except: pass
        self.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    rclpy.init()
    JogNode().run()
