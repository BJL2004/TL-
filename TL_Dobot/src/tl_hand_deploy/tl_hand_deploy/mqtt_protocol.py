"""MQTT command validation and the non-ROS MQTT client used by pick-place."""

from dataclasses import dataclass
import json
from queue import Empty, Queue
from threading import Event
from typing import Any


SUPPORTED_COMMANDS = {"seed", "trim_pick", "trim_place"}
RET_SUCCESS = 0
RET_FAILED = 1
RET_OCCUPIED = 2


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
    if "targetId" not in message or message["targetId"] in (None, ""):
        raise ValueError(f"MQTT cmd={command_name!r} requires targetId")
    return MqttCommand(name=str(command_name), target_id=str(message["targetId"]))


def encode_feedback(command_name: str, ret_code: int) -> str:
    if command_name not in SUPPORTED_COMMANDS:
        raise ValueError(f"unsupported feedback cmd={command_name!r}")
    if int(ret_code) not in (RET_SUCCESS, RET_FAILED, RET_OCCUPIED):
        raise ValueError(f"unsupported retCode={ret_code!r}")
    return json.dumps(
        {"cmd": command_name, "retCode": int(ret_code)},
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
                f"MQTT target map value for {target_id!r} must be an integer 1..6"
            ) from error
        if basket_number not in range(1, 7):
            raise ValueError(
                f"MQTT target map value for {target_id!r} must be a basket number 1..6"
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
        self.feedback: Queue[tuple[str, int]] = Queue()
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
                command_name, ret_code = self.feedback.get_nowait()
            except Empty:
                break
            if not self._publish_now(command_name, ret_code):
                self.feedback.put((command_name, ret_code))
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
        self.commands.put(command)

    def drain_commands(self) -> list[MqttCommand]:
        commands = []
        while True:
            try:
                commands.append(self.commands.get_nowait())
            except Empty:
                return commands

    def _publish_now(self, command_name: str, ret_code: int) -> bool:
        payload = encode_feedback(command_name, ret_code)
        result = self._client.publish(self.feedback_topic, payload, qos=1)
        if result.rc != 0:
            print(f"MQTT feedback publish failed: rc={result.rc}, payload={payload}")
            return False
        print(f"MQTT feedback: topic={self.feedback_topic}, payload={payload}")
        return True

    def publish_feedback(self, command_name: str, ret_code: int) -> None:
        if self._client is None or not self._connected:
            self.feedback.put((command_name, int(ret_code)))
            print(
                f"MQTT feedback queued until reconnect: "
                f"{encode_feedback(command_name, ret_code)}"
            )
            return
        if not self._publish_now(command_name, ret_code):
            self.feedback.put((command_name, int(ret_code)))

    def stop(self) -> None:
        if self._client is None:
            return
        self._client.loop_stop()
        self._client.disconnect()
        self._client = None
        self._connected = False
        self._connect_event.clear()
