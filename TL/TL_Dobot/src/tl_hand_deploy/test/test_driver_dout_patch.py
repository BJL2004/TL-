import importlib.util
from pathlib import Path
import tempfile
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[3]
PATCH_PATH = PROJECT_ROOT / "scripts" / "patch_tl_driver_dout_api.py"
SPEC = importlib.util.spec_from_file_location("patch_tl_driver_dout_api", PATCH_PATH)
PATCH_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PATCH_MODULE)


class DriverDoutPatchTest(unittest.TestCase):
    def test_patch_is_idempotent(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            interface = root / "tl_interface.py"
            node = root / "tl_driver_node.py"
            interface.write_text(
                "import time\n"
                "def queue_motion_set_status(*args): return 0\n"
                "def queue_motion_send_to_controller(*args): return 0\n"
                "def queue_motion_push_back_dout(*args): return 0\n"
                "\ndef queue_moveJ(\n    socketFd: int,\n):\n    pass\n",
                encoding="utf-8",
            )
            node.write_text(
                "def handler():\n"
                "            ret = tl_interface.set_digital_output(self.fd, request.port, request.value)\n",
                encoding="utf-8",
            )

            self.assertTrue(PATCH_MODULE.patch_interface(interface))
            self.assertTrue(PATCH_MODULE.patch_node(node))
            self.assertFalse(PATCH_MODULE.patch_interface(interface))
            self.assertFalse(PATCH_MODULE.patch_node(node))
            self.assertIn("def queue_dout(", interface.read_text(encoding="utf-8"))
            self.assertIn("tl_interface.queue_dout(", node.read_text(encoding="utf-8"))

    def test_patch_upgrades_previous_queue_adapter(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            interface = root / "tl_interface.py"
            node = root / "tl_driver_node.py"
            interface.write_text(
                "import time\n"
                "def queue_motion_set_status(*args): return 0\n"
                "def queue_motion_send_to_controller(*args): return 0\n"
                "def queue_motion_push_back_dout(*args): return 0\n"
                "\n# TL_Dobot queued digital-output compatibility\n"
                "def queue_dout(socketFd, port, value, startup_delay=2.0, wait_time=0.2):\n"
                "    return 0\n"
                "\ndef queue_moveJ(\n    socketFd: int,\n):\n    pass\n",
                encoding="utf-8",
            )
            node.write_text(
                "def handler():\n"
                "            # TL_Dobot queued digital-output compatibility\n"
                "            ret = tl_interface.queue_dout(\n"
                "                self.fd,\n"
                "                request.port,\n"
                "                bool(request.value),\n"
                "                startup_delay=2.0,\n"
                "                wait_time=0.2,\n"
                "            )\n",
                encoding="utf-8",
            )

            self.assertTrue(PATCH_MODULE.patch_interface(interface))
            self.assertTrue(PATCH_MODULE.patch_node(node))
            self.assertIn("close_delay", interface.read_text(encoding="utf-8"))
            self.assertIn("close_delay=2.0", node.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
