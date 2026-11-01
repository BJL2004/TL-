#!/usr/bin/env python3
"""测试: 物体位→放置位→回零"""
import rclpy, time, math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from tl_ros2_interface.srv import OpenServoJ

POS1 = [-12.186, -36.462, -20.653, -3.344, -51.359, -0.004]
POS2 = [82.118, -45.067, 43.976, -3.324, -14.125, -0.036]
ZERO = [0, 0, 0, 0, 0, 0]

class TestMove(Node):
    def __init__(self):
        super().__init__('test_move')
        self.latest_joint = None
        self.create_subscription(JointState, '/joint_states', self.cb, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/tl_driver/set_servoj_pos', 10)
        self.srv = self.create_client(OpenServoJ, '/tl_driver/open_servoj')
        self.srv.wait_for_service(5)
        req = OpenServoJ.Request()
        req.vmax = [10.]*7; req.amax = [3000.]*7; req.jmax = [50000.]*7
        self.srv.call_async(req)
        t0 = time.time()
        while self.latest_joint is None and time.time()-t0 < 5:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_joint:
            self.pos = [v*180./math.pi for v in self.latest_joint.position]
            self.pos = (self.pos + [0.]*7)[:7]
        else:
            self.pos = [0.]*7
        self.create_timer(0.01, self.timer_cb)
        self.traj = []; self.idx = 0; self.moving = False

    def cb(self, m): self.latest_joint = m

    def timer_cb(self):
        if self.moving and self.idx < len(self.traj):
            self.pos = self.traj[self.idx]
            self.idx += 1
        elif self.moving:
            self.moving = False
        m = Float64MultiArray()
        m.data = [float(v) for v in self.pos]
        self.pub.publish(m)

    def go(self, target, name=""):
        tgt = list(target) + [0.]*(7-len(target)); tgt = tgt[:7]
        start = list(self.pos)
        n = 200
        self.traj = [[start[i]+(tgt[i]-start[i])*s/n for i in range(7)] for s in range(1,n+1)]
        self.idx = 0; self.moving = True
        print(f"→ {name} {target} ...")
        while self.moving:
            rclpy.spin_once(self, timeout_sec=0.1)
        # 等实际关节到位
        t0 = time.time()
        while self.latest_joint and time.time()-t0 < 10:
            actual = [v*180./math.pi for v in self.latest_joint.position]
            err = max(abs(actual[i]-tgt[i]) for i in range(min(6,len(tgt))))
            if err < 1.0:
                break
            rclpy.spin_once(self, timeout_sec=0.05)
        time.sleep(0.5)
        self.log_pose(name)
        print(f"  到达")

    def log_pose(self, tag):
        if not self.latest_joint: return
        j = [round(v*180./math.pi,2) for v in self.latest_joint.position]
        print(f"  [{tag}] 实际关节: {j[:6]}")

def main():
    rclpy.init()
    n = TestMove()
    n.go(POS1, "物体位")
    n.go(POS2, "放置位")
    n.go(ZERO, "回零")
    print("完成")
    n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
