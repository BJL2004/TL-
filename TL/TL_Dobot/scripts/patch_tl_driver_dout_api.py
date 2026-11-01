#!/usr/bin/env python3
import argparse
import py_compile
import shutil
from pathlib import Path


INTERFACE_MARKER = "# TL_Dobot queued digital-output compatibility"
NODE_MARKER = "# TL_Dobot queued digital-output compatibility"

INTERFACE_CODE = r'''

# TL_Dobot queued digital-output compatibility
def queue_dout(
    socketFd: int,
    port: int,
    value: bool,
    startup_delay: float = 2.0,
    wait_time: float = 0.2,
    close_delay: float = 2.0,
) -> int:
    """通过单指令队列设置数字输出，不发送机械臂运动指令。"""
    if port <= 0:
        raise ValueError("digital output port must be positive")
    if startup_delay < 0 or wait_time < 0 or close_delay < 0:
        raise ValueError("queue delays must be non-negative")

    open_ret = queue_motion_set_status(socketFd, True)
    if open_ret != 0:
        return open_ret

    command_ret = 0
    close_ret = 0
    try:
        time.sleep(startup_delay)
        command_ret = queue_motion_push_back_dout(socketFd, port, bool(value))
        if command_ret == 0:
            command_ret = queue_motion_send_to_controller(socketFd, 1, False)
        if command_ret == 0:
            time.sleep(wait_time)
    finally:
        close_ret = queue_motion_set_status(socketFd, False)
        time.sleep(close_delay)

    return command_ret if command_ret != 0 else close_ret

'''

NODE_REPLACEMENT = '''            # TL_Dobot queued digital-output compatibility
            ret = tl_interface.queue_dout(
                self.fd,
                request.port,
                bool(request.value),
                startup_delay=2.0,
                wait_time=0.2,
                close_delay=2.0,
            )
'''

NODE_REPLACEMENT_V1 = '''            # TL_Dobot queued digital-output compatibility
            ret = tl_interface.queue_dout(
                self.fd,
                request.port,
                bool(request.value),
                startup_delay=2.0,
                wait_time=0.2,
            )
'''


def parse_args():
    parser = argparse.ArgumentParser(
        description="Patch the TL ARM driver to send digital outputs through a one-command queue"
    )
    parser.add_argument(
        "--driver-root",
        type=Path,
        default=Path.home() / "Desktop" / "tl_robot_ros2_py-arm",
    )
    return parser.parse_args()


def backup_once(path: Path) -> None:
    backup = path.with_suffix(path.suffix + ".before_dout_api")
    if not backup.exists():
        shutil.copy2(path, backup)


def patch_interface(path: Path) -> bool:
    text = path.read_text(encoding="utf-8")
    if INTERFACE_MARKER in text:
        start = text.index("\n" + INTERFACE_MARKER)
        end = text.index("\ndef queue_moveJ(\n", start)
        block = text[start:end]
        if "close_delay" in block:
            return False
        backup = path.with_suffix(path.suffix + ".before_dout_api_v2")
        if not backup.exists():
            shutil.copy2(path, backup)
        path.write_text(text[:start] + INTERFACE_CODE + text[end:], encoding="utf-8")
        return True
    required = [
        "def queue_motion_set_status(",
        "def queue_motion_send_to_controller(",
        "def queue_motion_push_back_dout(",
    ]
    missing = [name for name in required if name not in text]
    if missing:
        raise RuntimeError(f"unsupported tl_interface.py; missing: {missing}")
    anchor = "\ndef queue_moveJ(\n"
    if text.count(anchor) != 1:
        raise RuntimeError("unexpected tl_interface.py layout")
    backup_once(path)
    path.write_text(text.replace(anchor, INTERFACE_CODE + anchor), encoding="utf-8")
    return True


def patch_node(path: Path) -> bool:
    text = path.read_text(encoding="utf-8")
    if NODE_MARKER in text:
        if "close_delay=2.0" in text:
            return False
        if text.count(NODE_REPLACEMENT_V1) != 1:
            raise RuntimeError("unexpected old queued digital-output handler layout")
        backup = path.with_suffix(path.suffix + ".before_dout_api_v2")
        if not backup.exists():
            shutil.copy2(path, backup)
        path.write_text(text.replace(NODE_REPLACEMENT_V1, NODE_REPLACEMENT), encoding="utf-8")
        return True
    anchor = "            ret = tl_interface.set_digital_output(self.fd, request.port, request.value)\n"
    if text.count(anchor) != 1:
        raise RuntimeError("unexpected set_digital_output handler layout")
    backup_once(path)
    path.write_text(text.replace(anchor, NODE_REPLACEMENT), encoding="utf-8")
    return True


def main() -> None:
    arguments = parse_args()
    root = arguments.driver_root.expanduser().resolve()
    interfaces = [
        root / "src" / "tl_driver" / "lib" / "arm" / "tl_interface.py",
        root / "src" / "tl_driver" / "tl_driver" / "lib" / "tl_interface.py",
    ]
    node = root / "src" / "tl_driver" / "tl_driver" / "tl_driver_node.py"
    for path in [*interfaces, node]:
        if not path.exists():
            raise FileNotFoundError(path)

    changed = [patch_interface(path) for path in interfaces]
    changed.append(patch_node(node))
    for path in [*interfaces, node]:
        py_compile.compile(str(path), doraise=True)

    print(f"Patched driver root: {root}")
    print(f"Changed files: {sum(changed)}; backups use .before_dout_api suffix")
    print("Rebuild tl_driver and restart its node before testing DO3/DO4.")


if __name__ == "__main__":
    main()
