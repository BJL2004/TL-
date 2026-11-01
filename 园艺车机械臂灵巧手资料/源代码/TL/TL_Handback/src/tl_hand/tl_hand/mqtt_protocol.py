"""MQTT command validation and the non-ROS MQTT client used by pick-place."""

from dataclasses import dataclass
import json
from queue import Empty, Queue
import re
from threading import Event
from typing import Any


# seed 与 seed_place 都是"完整放种周期"（固定点取种->定位花盆->放置->回位），
# 只是反馈 cmd 名不同：seed_place 用于车端协议里的"放种子放置"命令
SUPPORTED_COMMANDS = {"seed", "seed_place", "trim_pick", "trim_place", "estop"}
RET_SUCCESS = 0
RET_FAILED = 1
RET_OCCUPIED = 2
# 车端 trim 协议：1=响应命令开始动作（先发），3=识别不成功/放置失败（终态）
RET_STARTED = 1
RET_RECOG_FAILED = 3

# Basket rack layout: 2 regions (A/B) x 4 rows x 24 baskets = 192 baskets.
# A1-01..A1-24 -> 1..24, A2-01..A2-24 -> 25..48, A3-01..A3-24 -> 49..72,
# A4-01..A4-24 -> 73..96, B1-01..B1-24 -> 97..120, B2-01..B2-24 -> 121..144,
# B3-01..B3-24 -> 145..168, B4-01..B4-24 -> 169..192.
_ROW_PREFIXES = {
    "A1": 0,
    "A2": 24,
    "A3": 48,
    "A4": 72,
    "B1": 96,
    "B2": 120,
    "B3": 144,
    "B4": 168,
}
_ROW_INDEX_RE = re.compile(r"^(A[1-4]|B[1-4])-(\d{1,2})$")


def target_id_to_basket_number(target_id: str) -> int | None:
    """Parse a rack-style targetId like 'A3-17' into basket number 1..192.

    Returns None when the targetId does not follow the rack naming rule, so
    callers can fall back to the explicit target map.
    """
    match = _ROW_INDEX_RE.match(str(target_id).strip().upper())
    if not match:
        return None
    row, index_text = match.group(1), match.group(2)
    index = int(index_text)
    if not 1 <= index <= 24:
        return None
    return int(_ROW_PREFIXES[row]) + index


@dataclass(frozen=True)
class MqttCommand:
    name: str
    target_id: str


def parse_command(payload: str | bytes) -> MqttCommand:
    try:
        if isinstance(payload, bytes):
            payload = payload.decode("utf-8")
        message = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError(f"MQTT payload is not valid UTF-8 JSON: {error}") from error
    if not isinstance(message, dict):
        raise ValueError("MQTT payload must be a JSON object")
    command_name = message.get("cmd")
    if command_name not in SUPPORTED_COMMANDS:
        raise ValueError(
            f"unsupported MQTT cmd={command_name!r}; "
            f"expected one of {sorted(SUPPORTED_COMMANDS)}"
        )
    if command_name == "estop":
        # 急停命令不带 targetId，收到即锁存急停标志
        return MqttCommand(name="estop", target_id="")
    if "targetId" not in message or message["targetId"] in (None, ""):
        raise ValueError(f"MQTT cmd={command_name!r} requires targetId")
    return MqttCommand(name=str(command_name), target_id=str(message["targetId"]))


def encode_feedback(
    command_name: str, ret_code: int, message: str = None, target_id: str = None
) -> str:
    if command_name not in SUPPORTED_COMMANDS:
        raise ValueError(f"unsupported feedback cmd={command_name!r}")
    if int(ret_code) not in (
        RET_SUCCESS,
        RET_FAILED,
        RET_STARTED,
        RET_OCCUPIED,
        RET_RECOG_FAILED,
    ):
        raise ValueError(f"unsupported retCode={ret_code!r}")
    if target_id is not None:
        payload = {
            "cmd": command_name,
            "targetId": str(target_id),
            "retCode": int(ret_code),
        }
    else:
        payload = {"cmd": command_name, "retCode": int(ret_code)}
    if message:
        payload["message"] = str(message)
    return json.dumps(
        payload,
        ensure_ascii=False,
        separators=(",", ":"),
    )


def parse_target_map(value: str | None) -> dict[str, int]:
    if not value:
        return {}
    try:
        raw_mapping: Any = json.loads(value)
    except json.JSONDecodeError as error:
        raise ValueError(f"TL_MQTT_TARGET_MAP must be a JSON object: {error}") from error
    if not isinstance(raw_mapping, dict):
        raise ValueError("TL_MQTT_TARGET_MAP must be a JSON object")
    mapping = {}
    for target_id, basket_number in raw_mapping.items():
        try:
            basket_number = int(basket_number)
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"MQTT target map value for {target_id!r} must be an integer 1..192"
            ) from error
        if basket_number not in range(1, 193):
            raise ValueError(
                f"MQTT target map value for {target_id!r} must be a basket number 1..192"
            )
        mapping[str(target_id)] = basket_number
    return mapping


class MqttClient:
    """Small paho client whose callbacks only enqueue commands for the ROS thread."""

    def __init__(
        self,
        host: str,
        port: int,
        command_topic: str,
        feedback_topic: str,
        client_id: str,
        username: str = "",
        password: str = "",
        keepalive: int = 60,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.command_topic = command_topic
        self.feedback_topic = feedback_topic
        self.client_id = client_id
        self.username = username
        self.password = password
        self.keepalive = int(keepalive)
        self.commands: Queue[MqttCommand] = Queue()
        self.feedback: Queue[tuple[str, int, str | None]] = Queue()
        # 由 MQTT 回调线程置位（无需等主循环），运动循环每帧检查后立即中止
        self.estop_requested = False
        self._client = None
        self._connected = False
        self._connect_event = Event()
        self._connect_error: str | None = None

    def start(self, timeout_sec: float = 10.0) -> None:
        try:
            import paho.mqtt.client as mqtt
        except ImportError as error:
            raise RuntimeError(
                "MQTT is enabled but paho-mqtt is missing; install "
                "python3-paho-mqtt or paho-mqtt on the Jetson"
            ) from error
        try:
            self._client = mqtt.Client(
                mqtt.CallbackAPIVersion.VERSION2,
                client_id=self.client_id,
            )
        except AttributeError:
            self._client = mqtt.Client(client_id=self.client_id)
        if self.username:
            self._client.username_pw_set(self.username, self.password)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.on_disconnect = self._on_disconnect
        self._connect_event.clear()
        self._connect_error = None
        self._client.connect_async(self.host, self.port, self.keepalive)
        self._client.loop_start()
        print(
            f"MQTT starting: broker={self.host}:{self.port}, "
            f"command={self.command_topic}, feedback={self.feedback_topic}"
        )
        if not self._connect_event.wait(timeout=max(0.1, float(timeout_sec))):
            self.stop()
            raise RuntimeError(
                f"MQTT broker {self.host}:{self.port} did not accept a connection "
                f"within {float(timeout_sec):.1f}s; check the broker, IP, port, "
                "firewall, and username/password"
            )
        if self._connect_error is not None:
            error = self._connect_error
            self.stop()
            raise RuntimeError(
                f"MQTT broker {self.host}:{self.port} rejected the connection: {error}"
            )

    def _on_connect(self, client, userdata, flags, reason_code, properties=None) -> None:
        reason_value = getattr(reason_code, "value", reason_code)
        if int(reason_value) != 0:
            self._connect_error = f"reason={reason_value}"
            self._connect_event.set()
            print(f"MQTT broker connection failed: {self._connect_error}")
            return
        self._connected = True
        client.subscribe(self.command_topic, qos=1)
        self._connect_event.set()
        print(f"MQTT connected and subscribed: {self.command_topic}")
        while True:
            try:
                command_name, ret_code, target_id = self.feedback.get_nowait()
            except Empty:
                break
            if not self._publish_now(command_name, ret_code, target_id=target_id):
                self.feedback.put((command_name, ret_code, target_id))
                break

    def _on_disconnect(
        self,
        client,
        userdata,
        disconnect_flags=None,
        reason_code=None,
        properties=None,
    ) -> None:
        self._connected = False
        reason_value = getattr(reason_code, "value", reason_code)
        print(f"MQTT disconnected: reason={reason_value}")

    def _on_message(self, client, userdata, message) -> None:
        try:
            command = parse_command(message.payload)
        except ValueError as error:
            print(f"MQTT command rejected: {error}")
            return
        if command.name == "estop":
            self.estop_requested = True
            print("MQTT ESTOP received: motion will abort at the next check point")
        self.commands.put(command)

    def drain_commands(self) -> list[MqttCommand]:
        commands = []
        while True:
            try:
                commands.append(self.commands.get_nowait())
            except Empty:
                return commands

    def _publish_now(
        self,
        command_name: str,
        ret_code: int,
        message: str = None,
        target_id: str = None,
    ) -> bool:
        payload = encode_feedback(command_name, ret_code, message, target_id)
        result = self._client.publish(self.feedback_topic, payload, qos=1)
        if result.rc != 0:
            print(f"MQTT feedback publish failed: rc={result.rc}, payload={payload}")
            return False
        print(f"MQTT feedback: topic={self.feedback_topic}, payload={payload}")
        return True

    def publish_feedback(
        self,
        command_name: str,
        ret_code: int,
        message: str = None,
        target_id: str = None,
    ) -> None:
        if self._client is None or not self._connected:
            self.feedback.put((command_name, int(ret_code), target_id))
            print(
                f"MQTT feedback queued until reconnect: "
                f"{encode_feedback(command_name, ret_code, message, target_id)}"
            )
            return
        if not self._publish_now(command_name, ret_code, message, target_id):
            self.feedback.put((command_name, int(ret_code), target_id))

    def stop(self) -> None:
        if self._client is None:
            return
        self._client.loop_stop()
        self._client.disconnect()
        self._client = None
        self._connected = False
        self._connect_event.clear()
