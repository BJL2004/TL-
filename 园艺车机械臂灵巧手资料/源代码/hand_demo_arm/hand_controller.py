#!/usr/bin/env python3
"""
傲意 ROH-LiteS001 灵巧手全功能控制模块

基于 ModBus-RTU 协议，通过天链机械臂 ModBus 透传端口2 (RS485) 控制。

寄存器映射:
  POS_TARGET  (1135-1140): 6指目标位置   uint16  0~65535
  POS_CURRENT (1145-1150): 6指当前位置   uint16  只读
  ANGLE_TARGET(1155-1160): 6指目标角度   int16   角度×100
  ANGLE_CURRENT(1165-1170):6指当前角度   int16   只读
  SPEED       (1125-1130): 6指逻辑速度   uint16  默认65535
  CURRENT     (1105-1110): 6指实时电流   uint16  mA, 只读
  CURRENT_LIM (1095-1100): 6指电流限制   uint16  mA
  FORCE_LIMIT (1115-1119): 5指力限制    uint16  mN (F0~F4)
  FORCE       (1120-1124): 5指当前力    uint16  mN (F0~F4), 只读
  STATUS      (1085-1090): 6指运动状态   uint16  只读
  FW_VERSION  (1001):     固件版本      uint16  只读
  NODE_ID     (1005):     从站地址      uint16  R/W
  RESET       (1014):     软复位       uint16  W
  START_INIT  (1013):     初始化       uint16  W
  SELF_TEST   (1008):     自检等级     uint16  R/W

手指编号:
  0: 拇指弯曲    1: 食指    2: 中指
  3: 无名指      4: 小指    5: 拇指旋转
"""

import time
from enum import IntEnum


# ============================================================
# 寄存器地址常量
# ============================================================

class Reg(IntEnum):
    """ModBus 寄存器地址 (起始基址)"""
    FW_VERSION       = 1001
    NODE_ID          = 1005
    SELF_TEST_LEVEL  = 1008
    BEEP_SWITCH      = 1009
    RESET            = 1014
    START_INIT       = 1013
    P_TARGET         = 1135   # POS_TARGET 起始
    P_CURRENT        = 1145
    ANGLE_TARGET     = 1155
    ANGLE_CURRENT    = 1165
    SPEED            = 1125
    CURRENT_LIMIT    = 1095
    CURRENT          = 1105
    FORCE_LIMIT      = 1115
    FORCE            = 1120
    STATUS           = 1085
    STALL_SPEED      = 1175
    STALL_CURRENT    = 1185
    STOP_PERIOD      = 1195
    RETRY_PERIOD     = 1205

FINGER_COUNT = 6

FINGER_NAMES = [
    "拇指弯曲", "食指", "中指",
    "无名指", "小指", "拇指旋转"
]

FINGER_SHORT = ["THUMB", "INDEX", "MIDDLE", "RING", "PINKY", "THUMB_ROT"]


# ============================================================
# 预设手势（位置值 0~65535，含义因校准而异，按实际微调）
# ============================================================

class Gesture:
    """预设手势定义。
    每个手势是一个 6 元素列表，对应 FINGER_NAMES 各手指的逻辑位置值。
    值含义:  0=完全张开, 65535=完全闭合 (方向因电机安装而异)
    """
    # 基础手势
    OPEN     = [    0,     0,     0,     0,     0,     0]  # 五指张开
    FIST     = [60000, 60000, 60000, 60000, 60000, 30000]  # 握拳
    HALF     = [30000, 30000, 30000, 30000, 30000, 15000]  # 半握

    # 抓取手势
    PINCH    = [20000, 20000,     0,     0,     0, 30000]  # 拇指+食指捏合
    TRIPOD   = [22000, 22000, 22000,     0,     0, 30000]  # 三指抓取
    POWER    = [60000, 60000, 60000, 60000, 60000,     0]  # 强力握 (拇指旋不转)

    # 实验手势
    POINT    = [    0,     0,     0,     0,     0,     0]  # 食指伸出（其余弯）
    VICTORY  = [30000,     0,     0, 30000, 30000, 15000]  # 比耶(拇/无名/小弯)
    OK       = [10000, 15000,     0,     0,     0, 20000]  # OK手势
    THUMB_UP = [    0, 60000, 60000, 60000, 60000, 60000]  # 竖大拇指

    # 自定义别名
    @classmethod
    def all(cls):
        return {k: v for k, v in vars(cls).items()
                if not k.startswith('_') and k != 'all' and isinstance(v, list)}

    @classmethod
    def get(cls, name):
        """按名称获取手势，支持中文名翻译"""
        table = {
            "张开": cls.OPEN, "握拳": cls.FIST, "半握": cls.HALF,
            "捏合": cls.PINCH, "三指": cls.TRIPOD, "强力握": cls.POWER,
            "比耶": cls.VICTORY, "OK": cls.OK, "竖拇指": cls.THUMB_UP,
        }
        if name in table:
            return table[name]
        return getattr(cls, name.upper(), None)


class FingerState:
    """单个手指的完整状态快照"""

    __slots__ = ('index', 'name', 'pos_target', 'pos_current',
                 'angle_target', 'angle_current', 'speed', 'current_ma',
                 'current_limit', 'force_mn', 'force_limit', 'status')

    def __init__(self, idx: int):
        self.index = idx
        self.name = FINGER_NAMES[idx]
        self.pos_target = 0
        self.pos_current = 0
        self.angle_target = 0
        self.angle_current = 0
        self.speed = 65535
        self.current_ma = 0
        self.current_limit = 0
        self.force_mn = 0
        self.force_limit = 0
        self.status = 0

    def __repr__(self):
        return (f"<{self.name}: pos={self.pos_current}/{self.pos_target} "
                f"angle={self.angle_current/100:.1f}° "
                f"I={self.current_ma}mA>")


# ============================================================
# 主控类
# ============================================================

class ROHHandController:
    """
    傲意 ROH-LiteS001 灵巧手 ModBus 控制器。

    用法:
        from hand_controller import ROHHandController

        hand = ROHHandController(node)  # node 是 ROS2 rclpy Node 实例

        hand.init()                     # 初始化 (读版本)
        hand.set_gesture("OPEN")        # 张开
        hand.set_gesture("FIST")        # 握拳
        hand.set_finger(1, 30000)       # 食指到 30000
        hand.set_fingers([0,10000,20000,30000,40000,50000])  # 6指独立
        state = hand.get_state()        # 读取全手状态
        for f in state:
            print(f.name, f.pos_current)
    """

    # ModBus 通信参数 (从 hand_demo control_node.yaml 读取)
    MASTER_ID   = 1
    SLAVE_ID    = 2
    PORT        = 2
    BAUDRATE    = 115200
    DATA_BIT    = 8
    STOP_BIT    = 1
    CHECK_BIT   = "N"

    def __init__(self, ros_node):
        """
        ros_node: 一个已初始化的 rclpy.Node 实例
                 必须已创建 modbus_write 和 modbus_read 服务客户端
        """
        self.node = ros_node
        self.log = ros_node.get_logger()
        self.fw_version = None

        # 缓存最新状态
        self.fingers = [FingerState(i) for i in range(FINGER_COUNT)]

        # 需要 ROS Node 提供这两个服务客户端
        self._modbus_write_client = None
        self._modbus_read_client = None

        # 尝试自动发现服务客户端
        from tl_ros2_interface.srv import ModbusWrite, ModbusRead
        for attr in dir(ros_node):
            obj = getattr(ros_node, attr, None)
            if hasattr(obj, 'srv_type'):
                if obj.srv_type == ModbusWrite:
                    self._modbus_write_client = obj
                elif obj.srv_type == ModbusRead:
                    self._modbus_read_client = obj

    @property
    def modbus_write_client(self):
        if self._modbus_write_client is None:
            from tl_ros2_interface.srv import ModbusWrite
            self._modbus_write_client = self.node.create_client(
                ModbusWrite, '/tl_driver/modbus_write'
            )
        return self._modbus_write_client

    @property
    def modbus_read_client(self):
        if self._modbus_read_client is None:
            from tl_ros2_interface.srv import ModbusRead
            self._modbus_read_client = self.node.create_client(
                ModbusRead, '/tl_driver/modbus_read'
            )
        return self._modbus_read_client

    # ── 服务调用 ──────────────────────────────────────────────

    def _call_write(self, addr: int, data: list, timeout=5.0):
        """写多个保持寄存器 (功能码 0x10)"""
        import rclpy
        from tl_ros2_interface.srv import ModbusWrite
        from tl_ros2_interface.msg import ModbusMasterParam, ModbusRTUParam

        req = ModbusWrite.Request()
        req.master_id = self.MASTER_ID
        req.addr = addr
        req.data = [int(v) for v in data]

        rtu = ModbusRTUParam()
        rtu.slave_id = self.SLAVE_ID
        rtu.port = self.PORT
        rtu.baudrate = self.BAUDRATE
        rtu.data_bit = self.DATA_BIT
        rtu.stop_bit = self.STOP_BIT
        rtu.check_bit = self.CHECK_BIT

        param = ModbusMasterParam()
        param.type = "RTU"
        param.start_addr = True
        param.rtu = rtu

        req.master_param = param

        if not self.modbus_write_client.wait_for_service(timeout_sec=timeout):
            self.log.error("modbus_write 服务不可用")
            return False

        future = self.modbus_write_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        if future.done() and future.result() and future.result().success:
            return True
        msg = future.result().message if future.done() and future.result() else "timeout"
        self.log.error(f"modbus_write({addr}): {msg}")
        return False

    def _call_read(self, addr: int, count: int, timeout=5.0):
        """读多个保持寄存器 (功能码 0x03)"""
        import rclpy
        from tl_ros2_interface.srv import ModbusRead
        from tl_ros2_interface.msg import ModbusMasterParam, ModbusRTUParam

        req = ModbusRead.Request()
        req.master_id = self.MASTER_ID
        req.addr = addr
        req.quantity = count

        rtu = ModbusRTUParam()
        rtu.slave_id = self.SLAVE_ID
        rtu.port = self.PORT
        rtu.baudrate = self.BAUDRATE
        rtu.data_bit = self.DATA_BIT
        rtu.stop_bit = self.STOP_BIT
        rtu.check_bit = self.CHECK_BIT

        param = ModbusMasterParam()
        param.type = "RTU"
        param.start_addr = True
        param.rtu = rtu

        req.master_param = param

        if not self.modbus_read_client.wait_for_service(timeout_sec=timeout):
            self.log.error("modbus_read 服务不可用")
            return None

        future = self.modbus_read_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        if future.done() and future.result() and future.result().success:
            return list(future.result().data)
        msg = future.result().message if future.done() and future.result() else "timeout"
        self.log.error(f"modbus_read({addr}): {msg}")
        return None

    # ── 初始化 ────────────────────────────────────────────────

    def init(self):
        """初始化灵巧手：读取固件版本，确认通信正常"""
        data = self._call_read(Reg.FW_VERSION, 1)
        if data and len(data) > 0:
            self.fw_version = data[0]
            self.log.info(f"灵巧手固件版本: {self.fw_version}")
            self.log.info(f"灵巧手初始化完成，支持6指独立控制")
            return True
        self.log.error("灵巧手初始化失败：无法读取固件版本")
        return False

    def reset(self):
        """软复位灵巧手"""
        return self._call_write(Reg.RESET, [1])

    def start_init(self):
        """触发自检初始化（手会咔咔响，正常）"""
        self._call_write(Reg.SELF_TEST_LEVEL, [1])
        time.sleep(0.1)
        return self._call_write(Reg.START_INIT, [1])

    def beep(self, on=True):
        """蜂鸣器开关"""
        return self._call_write(Reg.BEEP_SWITCH, [1 if on else 0])

    # ── 基础控制 ──────────────────────────────────────────────

    def set_finger(self, index: int, position: int):
        """
        控制单个手指到指定逻辑位置。
        index: 0~5 (拇指弯曲~拇指旋转)
        position: uint16, 0=完全张开, 65535=完全闭合
        """
        if not 0 <= index < FINGER_COUNT:
            self.log.error(f"手指编号 {index} 超出范围 0~5")
            return False
        addr = Reg.P_TARGET + index
        return self._call_write(addr, [int(position)])

    def set_fingers(self, positions: list):
        """
        同时控制6个手指到指定位置。
        positions: 6个 uint16 数组
        """
        if len(positions) != FINGER_COUNT:
            self.log.error(f"需要 {FINGER_COUNT} 个位置值，收到 {len(positions)}")
            return False
        return self._call_write(Reg.P_TARGET, [int(p) for p in positions])

    def set_gesture(self, name: str):
        """执行预设手势，如 'OPEN', 'FIST', 'PINCH' 等"""
        gesture = Gesture.get(name)
        if gesture is None:
            self.log.error(f"未知手势: {name}, 可用: {list(Gesture.all().keys())}")
            return False
        self.log.info(f"执行手势: {name} → {[f'{v}' for v in gesture]}")
        return self.set_fingers(gesture)

    # ── 速度控制 ──────────────────────────────────────────────

    def set_speed(self, speeds):
        """
        设置各手指运动速度。
        speeds: int (所有手指统一) 或 6元素列表
        """
        if isinstance(speeds, (int, float)):
            speeds = [int(speeds)] * FINGER_COUNT
        if len(speeds) != FINGER_COUNT:
            self.log.error(f"需要 {FINGER_COUNT} 个速度值")
            return False
        return self._call_write(Reg.SPEED, [int(s) for s in speeds])

    def set_speed_fast(self):
        """最大速度 (65535)"""
        return self.set_speed(65535)

    def set_speed_slow(self):
        """慢速 (20000)"""
        return self.set_speed(20000)

    # ── 电流/力限制 ───────────────────────────────────────────

    def set_current_limit(self, limits):
        """设置各手指电流限制 (mA), int 或 6元素列表"""
        if isinstance(limits, (int, float)):
            limits = [int(limits)] * FINGER_COUNT
        return self._call_write(Reg.CURRENT_LIMIT, [int(v) for v in limits])

    def set_force_limit(self, limits):
        """设置5指力限制 (mN, F0~F4 不包括拇指旋转)"""
        if isinstance(limits, (int, float)):
            limits = [int(limits)] * 5
        return self._call_write(Reg.FORCE_LIMIT, [int(v) for v in limits])

    # ── 状态读取 ──────────────────────────────────────────────

    def get_positions(self):
        """读取6指当前位置 → list[int] 或 None"""
        data = self._call_read(Reg.P_CURRENT, FINGER_COUNT)
        if data and len(data) >= FINGER_COUNT:
            for i in range(FINGER_COUNT):
                self.fingers[i].pos_current = data[i]
            return data[:FINGER_COUNT]
        return None

    def get_currents(self):
        """读取6指实时电流 (mA) → list[int] 或 None"""
        data = self._call_read(Reg.CURRENT, FINGER_COUNT)
        return data[:FINGER_COUNT] if data and len(data) >= FINGER_COUNT else None

    def get_forces(self):
        """读取5指实时力 (mN) → list[int] 或 None"""
        data = self._call_read(Reg.FORCE, 5)
        return data if data and len(data) >= 5 else None

    def get_status(self):
        """读取6指运动状态 → list[int] 或 None。0=空闲, 非0=运动中/故障"""
        data = self._call_read(Reg.STATUS, FINGER_COUNT)
        return data[:FINGER_COUNT] if data and len(data) >= FINGER_COUNT else None

    def get_state(self):
        """
        读取全手完整状态，返回 6 个 FingerState 对象列表。
        同时更新内部缓存 self.fingers。
        """
        pos = self.get_positions()
        cur = self.get_currents()
        status = self.get_status()

        for i in range(FINGER_COUNT):
            if pos:  self.fingers[i].pos_current = pos[i]
            if cur:  self.fingers[i].current_ma = cur[i]
            if status: self.fingers[i].status = status[i]

        return self.fingers

    def is_moving(self):
        """是否有手指正在运动"""
        status = self.get_status()
        if status is None:
            return False
        return any(s != 0 for s in status)

    def wait_stop(self, timeout=10.0, poll_interval=0.1):
        """等待所有手指停止运动"""
        start = time.time()
        while time.time() - start < timeout:
            if not self.is_moving():
                return True
            time.sleep(poll_interval)
        self.log.warn(f"等待手指停止超时 ({timeout}s)")
        return False

    # ── 便捷方法 ──────────────────────────────────────────────

    def open(self):
        """快捷: 张开手"""
        return self.set_gesture("OPEN")

    def close(self):
        """快捷: 握拳"""
        return self.set_gesture("FIST")

    def pinch(self):
        """快捷: 捏合"""
        return self.set_gesture("PINCH")

    def grasp_and_wait(self, gesture="FIST", timeout=5.0):
        """执行手势并等待完成"""
        self.set_gesture(gesture)
        return self.wait_stop(timeout)


# ============================================================
# 独立测试入口 (也可作为 ROS2 Node 独立运行)
# ============================================================

def main():
    """独立测试灵巧手"""
    import rclpy
    from rclpy.node import Node

    class HandTestNode(Node):
        def __init__(self):
            super().__init__("hand_test")
            self.hand = ROHHandController(self)

    rclpy.init()
    tester = HandTestNode()

    if not tester.hand.init():
        tester.get_logger().error("初始化失败，退出")
        rclpy.shutdown()
        return

    tester.get_logger().info("=== 灵巧手测试 ===")

    # 测试手势
    import sys
    if len(sys.argv) > 1:
        name = sys.argv[1]
        tester.hand.set_gesture(name)
    else:
        # 完整测试
        tests = [
            ("张开", "OPEN"),
            ("握拳", "FIST"),
            ("半握", "HALF"),
            ("捏合", "PINCH"),
            ("三指抓", "TRIPOD"),
            ("张开", "OPEN"),
        ]
        for label, gesture in tests:
            tester.get_logger().info(f"--- {label} ---")
            tester.hand.set_gesture(gesture)
            time.sleep(1.5)

        # 读取状态
        tester.get_logger().info("=== 读取全手状态 ===")
        state = tester.hand.get_state()
        for f in state:
            tester.get_logger().info(
                f"  {f.name}: pos={f.pos_current}, current={f.current_ma}mA, "
                f"status={f.status}"
            )

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
