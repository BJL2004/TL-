#!/usr/bin/env python3
import argparse
import py_compile
import shutil
from pathlib import Path


INTERFACE_MARKER = "# TL_Dobot coordinate-number compatibility"
NODE_MARKER = "# TL_Dobot SetCoordNum compatibility"

INTERFACE_CODE = r'''

# TL_Dobot coordinate-number compatibility
def _coord_number_func(c_name: str, cpp_name: str, argtypes):
    func = getattr(_lib, c_name + "_c", None)
    if func is None:
        func = getattr(_lib, cpp_name, None)
    if func is None:
        raise AttributeError(f"Function {c_name} not found in library")
    func.argtypes = argtypes
    func.restype = ctypes.c_int
    return func


def get_tool_hand_number(socketFd: int, initial_value: int = -1) -> tuple:
    value = ctypes.c_int(initial_value)
    func = _coord_number_func(
        "get_tool_hand_number",
        "_Z20get_tool_hand_numberiRi",
        [SOCKETFD, ctypes.POINTER(ctypes.c_int)],
    )
    ret = func(socketFd, ctypes.byref(value))
    return ret, value.value


def set_tool_hand_number(socketFd: int, tool_num: int) -> int:
    func = _coord_number_func(
        "set_tool_hand_number",
        "_Z20set_tool_hand_numberii",
        [SOCKETFD, ctypes.c_int],
    )
    return func(socketFd, tool_num)


def get_user_coord_number(socketFd: int, initial_value: int = -1) -> tuple:
    value = ctypes.c_int(initial_value)
    func = _coord_number_func(
        "get_user_coord_number",
        "_Z21get_user_coord_numberiRi",
        [SOCKETFD, ctypes.POINTER(ctypes.c_int)],
    )
    ret = func(socketFd, ctypes.byref(value))
    return ret, value.value


def set_user_coord_number(socketFd: int, user_num: int) -> int:
    func = _coord_number_func(
        "set_user_coord_number",
        "_Z21set_user_coord_numberii",
        [SOCKETFD, ctypes.c_int],
    )
    return func(socketFd, user_num)
'''

SERVICE_CODE = '''            # TL_Dobot SetCoordNum compatibility
            self.create_service(
                srvs.SetCoordNum,
                "/tl_driver/set_coord_num",
                self.handle_set_coord_num_service,
                callback_group=self.service_group_,
            )
'''

HANDLER_CODE = '''    # TL_Dobot SetCoordNum compatibility
    def handle_set_coord_num_service(self, request, response):
        if self.fd is None or not self.is_connected_:
            response.success = False
            response.message = "Arm is not connected"
            return response
        if request.tool_num < 0 or request.user_num < 0:
            response.success = False
            response.message = "tool_num and user_num must be non-negative"
            return response
        try:
            ret_tool = tl_interface.set_tool_hand_number(self.fd, request.tool_num)
            ret_user = tl_interface.set_user_coord_number(self.fd, request.user_num)
            check_tool, tool_num = tl_interface.get_tool_hand_number(self.fd, -1)
            check_user, user_num = tl_interface.get_user_coord_number(self.fd, -1)
            response.success = (
                ret_tool == 0
                and ret_user == 0
                and check_tool == 0
                and check_user == 0
                and tool_num == request.tool_num
                and user_num == request.user_num
            )
            response.message = (
                f"tool={tool_num}, user={user_num}, "
                f"set_ret=({ret_tool},{ret_user}), get_ret=({check_tool},{check_user})"
            )
        except Exception as error:
            response.success = False
            response.message = str(error)
            self.get_logger().error(f"handle_set_coord_num_service failed: {error}")
        return response

'''


def parse_args():
    parser = argparse.ArgumentParser(description="Patch missing ARM64 coordinate-number APIs")
    parser.add_argument(
        "--driver-root",
        type=Path,
        default=Path.home() / "Desktop" / "tl_robot_ros2_py-arm",
    )
    return parser.parse_args()


def backup_once(path: Path) -> None:
    backup = path.with_suffix(path.suffix + ".before_coord_api")
    if not backup.exists():
        shutil.copy2(path, backup)


def patch_interface(path: Path, library_path: Path) -> bool:
    text = path.read_text(encoding="utf-8")
    if INTERFACE_MARKER in text:
        return False
    library = library_path.read_bytes()
    required_symbols = [
        b"_Z20get_tool_hand_numberiRi",
        b"_Z20set_tool_hand_numberii",
        b"_Z21get_user_coord_numberiRi",
        b"_Z21set_user_coord_numberii",
    ]
    missing = [symbol.decode() for symbol in required_symbols if symbol not in library]
    if missing:
        raise RuntimeError(f"vendor library is missing expected symbols: {missing}")
    anchor = "\ndef get_current_mode(socketFd: int) -> int:\n"
    if text.count(anchor) != 1:
        raise RuntimeError("unexpected tl_interface.py layout")
    backup_once(path)
    path.write_text(text.replace(anchor, INTERFACE_CODE + anchor), encoding="utf-8")
    return True


def patch_node(path: Path) -> bool:
    text = path.read_text(encoding="utf-8")
    if NODE_MARKER in text:
        return False
    service_anchor = '''            self.create_service(
                srvs.SetCurrentCoord,
'''
    handler_anchor = "    def handle_set_current_coord_service(self, request, response):\n"
    if text.count(service_anchor) != 1 or text.count(handler_anchor) != 1:
        raise RuntimeError("unexpected tl_driver_node.py layout")
    backup_once(path)
    text = text.replace(service_anchor, SERVICE_CODE + service_anchor)
    text = text.replace(handler_anchor, HANDLER_CODE + handler_anchor)
    path.write_text(text, encoding="utf-8")
    return True


def main() -> None:
    arguments = parse_args()
    root = arguments.driver_root.expanduser().resolve()
    interface = root / "src" / "tl_driver" / "lib" / "arm" / "tl_interface.py"
    node = root / "src" / "tl_driver" / "tl_driver" / "tl_driver_node.py"
    library = root / "src" / "tl_driver" / "lib" / "arm" / "_tl_host.so"
    for path in [interface, node, library]:
        if not path.exists():
            raise FileNotFoundError(path)
    changed = [patch_interface(interface, library), patch_node(node)]
    py_compile.compile(str(interface), doraise=True)
    py_compile.compile(str(node), doraise=True)
    print(f"Patched driver root: {root}")
    print(f"Changed files: {sum(changed)}; backups use .before_coord_api suffix")
    print("Rebuild tl_driver and restart the driver node before calling coordinate services.")


if __name__ == "__main__":
    main()
