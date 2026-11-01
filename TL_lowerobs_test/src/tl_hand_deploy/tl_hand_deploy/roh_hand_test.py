#!/usr/bin/env python3
"""ROH-LiteS001 灵巧手独立测试（参考 TL_TCB 工程 test_hand.py / grasp_test.py）

通过天链机械臂 ModBus 透传端口2 (RS485) 控制灵巧手。

按键:
  z  初始化     r 复位     o 张开
  1-6 抓取力度分级（极轻/轻握/中握/重握/强力/握拳）
  v 比耶  m 竖中指  p 捏合  k OK  g 赞  s 摇滚  i 食指指
  q 退出

每次命令后自动显示 6 指位置与电流。
"""

import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from tl_ros2_interface.msg import ModbusMasterParam, ModbusRTUParam
from tl_ros2_interface.srv import ModbusRead, ModbusWrite

MASTER_ID = 1
SLAVE_ID = 2
PORT = 2
BAUDRATE = 115200

REG_FW = 1001
REG_SELF_TEST = 1008
REG_RESET = 1014
REG_START_INIT = 1013
REG_STATUS = 1085
REG_CURRENT_MA = 1105
REG_POS_TARGET = 1135
REG_POS = 1145

FINGERS = ["拇指弯", "食指", "中指", "无名指", "小指", "拇指旋"]
OPEN_POS = [0, 0, 0, 0, 0, 0]

# 抓取力度分级（位置值 0~65535, 0=全开）
GRASP_LEVELS = {
    "1": ("极轻", [8000, 8000, 8000, 8000, 8000, 5000]),
    "2": ("轻握", [12000, 12000, 12000, 12000, 12000, 8000]),
    "3": ("中握", [18000, 18000, 18000, 18000, 18000, 12000]),
    "4": ("重握", [25000, 25000, 25000, 25000, 25000, 18000]),
    "5": ("强力", [35000, 35000, 35000, 35000, 35000, 25000]),
    "6": ("握拳", [50000, 50000, 50000, 50000, 50000, 30000]),
}

# 手势预设（参考 TL_TCB test_hand.py）
GESTURES = {
    "v": ("比耶", [30000, 0, 0, 60000, 60000, 15000]),
    "m": ("竖中指", [60000, 60000, 0, 60000, 60000, 0]),
    "p": ("捏合", [20000, 20000, 0, 0, 0, 30000]),
    "k": ("OK", [10000, 15000, 0, 0, 0, 20000]),
    "g": ("赞", [0, 60000, 60000, 60000, 60000, 60000]),
    "s": ("摇滚", [30000, 0, 60000, 60000, 0, 15000]),
    "i": ("食指指", [60000, 0, 60000, 60000, 60000, 0]),
}


class RohHandTest(Node):
    def __init__(self):
        super().__init__("roh_hand_test")
        self.write_client = self.create_client(
            ModbusWrite, "/tl_driver/modbus_write"
        )
        self.read_client = self.create_client(
            ModbusRead, "/tl_driver/modbus_read"
        )
        self.fd = sys.stdin.fileno()
        self.old_termios = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

    def _restore_terminal(self):
        try:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_termios)
        except Exception:
            pass

    def _make_master_param(self) -> ModbusMasterParam:
        rtu = ModbusRTUParam()
        rtu.slave_id = SLAVE_ID
        rtu.port = PORT
        rtu.baudrate = BAUDRATE
        rtu.data_bit = 8
        rtu.stop_bit = 1
        rtu.check_bit = "None"
        master = ModbusMasterParam()
        master.type = "RTU"
        master.start_addr = True
        master.rtu = rtu
        return master

    def modbus_write(self, addr: int, data) -> bool:
        request = ModbusWrite.Request()
        request.master_id = MASTER_ID
        request.addr = addr
        request.data = [int(value) for value in data]
        request.master_param = self._make_master_param()
        if not self.write_client.wait_for_service(timeout_sec=3.0):
            print("❌ /tl_driver/modbus_write 不可用（驱动启动了吗？）")
            return False
        future = self.write_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        return bool(
            future.done() and future.result() and future.result().success
        )

    def modbus_read(self, addr: int, count: int):
        request = ModbusRead.Request()
        request.master_id = MASTER_ID
        request.addr = addr
        request.quantity = count
        request.master_param = self._make_master_param()
        if not self.read_client.wait_for_service(timeout_sec=3.0):
            print("❌ /tl_driver/modbus_read 不可用（驱动启动了吗？）")
            return None
        future = self.read_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.done() and future.result() and future.result().success:
            return list(future.result().data)
        return None

    def show_state(self) -> None:
        positions = self.modbus_read(REG_POS, 6)
        currents = self.modbus_read(REG_CURRENT_MA, 6)
        if positions:
            print(
                "  位置: "
                + " ".join(
                    f"{FINGERS[i]}={positions[i]}" for i in range(len(positions))
                )
            )
        if currents:
            print(
                "  电流: "
                + " ".join(
                    f"{FINGERS[i]}={currents[i]}mA" for i in range(len(currents))
                )
            )

    def init_hand(self) -> None:
        print("初始化灵巧手（自检）...")
        self.modbus_write(REG_SELF_TEST, [1])
        time.sleep(0.1)
        self.modbus_write(REG_START_INIT, [1])
        time.sleep(2.0)
        firmware = self.modbus_read(REG_FW, 1)
        if firmware and len(firmware) > 0:
            print(f"✅ 灵巧手就绪（固件={firmware[0]}）")
        else:
            print("⚠️ 无法读取固件版本，检查 RS485 连接")

    def reset_hand(self) -> None:
        print("软复位灵巧手...")
        self.modbus_write(REG_RESET, [1])
        time.sleep(1.0)

    def move(self, label: str, positions) -> None:
        ok = self.modbus_write(REG_POS_TARGET, positions)
        print(f"{'✅' if ok else '❌'} {label}: {positions}")
        time.sleep(0.8)
        self.show_state()

    def run(self) -> None:
        print("ROH-LiteS001 灵巧手测试")
        print(
            "按键: z初始化 r复位 o张开 | 1-6力度 | "
            "v比耶 m中指 p捏合 kOK g赞 s摇滚 i指 | q退出"
        )
        while rclpy.ok():
            ready, _, _ = select.select([self.fd], [], [], 0.1)
            if not ready:
                continue
            try:
                key = os.read(self.fd, 1).decode(errors="ignore").lower()
            except OSError:
                continue
            if key in ("q", "\x03"):
                break
            elif key == "z":
                self.init_hand()
            elif key == "r":
                self.reset_hand()
            elif key == "o":
                self.move("张开", OPEN_POS)
            elif key in GRASP_LEVELS:
                label, positions = GRASP_LEVELS[key]
                self.move(f"抓取-{label}", positions)
            elif key in GESTURES:
                label, positions = GESTURES[key]
                self.move(label, positions)


def main(argv=None):
    rclpy.init()
    node = RohHandTest()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node._restore_terminal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
