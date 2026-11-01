#!/usr/bin/env python3
"""
灵巧手抓取力分级测试
====================
按键切换不同抓取力等级，逐步找到既不空转又不堵转的合适值。
支持实时显示各手指位置和电流。

用法:
  source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash
  python3 grasp_test.py
"""

import sys, os, time, select, termios, tty, threading
import rclpy
from rclpy.node import Node
from tl_ros2_interface.srv import ModbusWrite, ModbusRead
from tl_ros2_interface.msg import ModbusMasterParam, ModbusRTUParam

# ============================================================
# 抓取力等级预设（位置值 0~65535, 0=全开）
# ============================================================
GRASP_LEVELS = {
    # 键  名称        拇指  食指  中指  无名指 小指  拇指旋转
    '1': ("极轻",    [ 8000,  8000,  8000,  8000,  8000,  5000]),
    '2': ("轻握",    [12000, 12000, 12000, 12000, 12000,  8000]),
    '3': ("中握",    [18000, 18000, 18000, 18000, 18000, 12000]),
    '4': ("重握",    [25000, 25000, 25000, 25000, 25000, 18000]),
    '5': ("强力",    [35000, 35000, 35000, 35000, 35000, 25000]),
    '6': ("握拳",    [50000, 50000, 50000, 50000, 50000, 30000]),
}

OPEN_POS = [0, 0, 0, 0, 0, 0]
FINGER_NAMES = ["拇指弯", "食指", "中指", "无名指", "小指", "拇指旋"]

# ModBus 参数
MASTER_ID = 1
SLAVE_ID = 2
PORT = 2
BAUDRATE = 115200
POS_TARGET = 1135
POS_CURRENT = 1145
CURRENT_MA = 1105
STATUS = 1085


class GraspTestNode(Node):
    def __init__(self):
        super().__init__('grasp_test')
        self.write_client = self.create_client(ModbusWrite, '/tl_driver/modbus_write')
        self.read_client = self.create_client(ModbusRead, '/tl_driver/modbus_read')
        self.current_level = None
        self.running = True
        self._setup_kb()

    def _setup_kb(self):
        try:
            self.kb_fd = sys.stdin.fileno() if sys.stdin.isatty() else os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
            self.kb_old = termios.tcgetattr(self.kb_fd)
            tty.setcbreak(self.kb_fd)
        except:
            self.kb_fd = None

    def _read_key(self):
        if self.kb_fd is None:
            return None
        try:
            dr, _, _ = select.select([self.kb_fd], [], [], 0)
            if dr:
                return os.read(self.kb_fd, 1).decode(errors='ignore').lower()
        except:
            pass
        return None

    def _make_param(self):
        rtu = ModbusRTUParam()
        rtu.slave_id = SLAVE_ID
        rtu.port = PORT
        rtu.baudrate = BAUDRATE
        rtu.data_bit = 8
        rtu.stop_bit = 1
        rtu.check_bit = "None"
        p = ModbusMasterParam()
        p.type = "RTU"
        p.start_addr = True
        p.rtu = rtu
        return p

    def modbus_write(self, addr, data):
        req = ModbusWrite.Request()
        req.master_id = MASTER_ID
        req.addr = addr
        req.data = [int(v) for v in data]
        req.master_param = self._make_param()
        if not self.write_client.wait_for_service(timeout_sec=3):
            return False
        f = self.write_client.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=3)
        return f.done() and f.result() and f.result().success

    def modbus_read(self, addr, count):
        req = ModbusRead.Request()
        req.master_id = MASTER_ID
        req.addr = addr
        req.quantity = count
        req.master_param = self._make_param()
        if not self.read_client.wait_for_service(timeout_sec=3):
            return None
        f = self.read_client.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=3)
        if f.done() and f.result() and f.result().success:
            return list(f.result().data)
        return None

    def init_hand(self):
        """自检初始化灵巧手"""
        print("正在初始化灵巧手...")
        # SELF_TEST_LEVEL = 1
        if not self.modbus_write(1008, [1]):
            print("❌ 写 SELF_TEST_LEVEL 失败")
            return False
        time.sleep(0.1)
        # START_INIT = 1
        if not self.modbus_write(1013, [1]):
            print("❌ 写 START_INIT 失败")
            return False
        time.sleep(2)
        # 验证固件版本
        fw = self.modbus_read(1001, 1)
        if fw and len(fw) > 0:
            print(f"✅ 灵巧手就绪 (固件={fw[0]})")
            return True
        print("❌ 无法读取固件版本")
        return False

    def grasp(self, positions):
        """写入 6 指目标位置"""
        name = [k for k, v in GRASP_LEVELS.items() if v[1] == positions]
        label = GRASP_LEVELS[name[0]][0] if name else "自定义"
        ok = self.modbus_write(POS_TARGET, positions)
        if ok:
            self.current_level = positions
            print(f"\n👉 {label}: {positions}")
        else:
            print(f"\n❌ 写入失败")
        return ok

    def open_hand(self):
        self.grasp(OPEN_POS)
        self.current_level = OPEN_POS

    def read_state(self):
        """读取各手指位置和电流"""
        pos = self.modbus_read(POS_CURRENT, 6)
        cur = self.modbus_read(CURRENT_MA, 6)
        sta = self.modbus_read(STATUS, 6)

        if not pos:
            return

        print("\n手指状态:")
        print(f"{'手指':<8} {'位置':>6} {'电流mA':>6} {'状态':>4}")
        print("-" * 30)
        for i in range(6):
            p = pos[i] if pos and i < len(pos) else 0
            c = cur[i] if cur and i < len(cur) else 0
            s = sta[i] if sta and i < len(sta) else 0
            s_text = {0: "空闲", 1: "运动中", 2: "到位", 3: "过流", 4: "力限", 5: "堵转"}.get(s, str(s))
            bar = "█" * min(20, int(p / 65535 * 20))
            print(f"{FINGER_NAMES[i]:<8} {p:>5d} {c:>5d}mA {s_text:<4} {bar}")

    def run(self):
        # 初始化
        if not self.read_client.wait_for_service(timeout_sec=10):
            print("❌ tl_driver 未运行，请先启动驱动")
            return
        if not self.init_hand():
            return

        self.open_hand()
        time.sleep(1)

        print("\n" + "=" * 58)
        print("  灵巧手抓取力分级测试")
        print("=" * 58)
        for k, (name, vals) in GRASP_LEVELS.items():
            print(f"  [{k}] {name:<6}  {vals}")
        print(f"  [o] 张开")
        print(f"  [s] 读取手指状态")
        print(f"  [q] 退出")
        print("=" * 58)
        print("⚠️  从 [1] 极轻开始，听到电机异响立刻松键!")

        try:
            while self.running:
                rclpy.spin_once(self, timeout_sec=0.05)
                k = self._read_key()
                if k is None:
                    continue
                if k in GRASP_LEVELS:
                    self.grasp(GRASP_LEVELS[k][1])
                elif k == 'o':
                    self.open_hand()
                    print("\n👉 张开")
                elif k == 's':
                    self.read_state()
                elif k == 'q':
                    self.running = False
        finally:
            self.open_hand()
            try:
                termios.tcsetattr(self.kb_fd, termios.TCSADRAIN, self.kb_old)
            except:
                pass
            self.destroy_node()
            rclpy.shutdown()
            print("\n退出")


def main():
    rclpy.init()
    node = GraspTestNode()
    node.run()


if __name__ == '__main__':
    main()
