import rclpy
from tl_driver.tl_driver.tl_driver_node import TLArmNode
from std_srvs.srv import Trigger
import tl_ros2_interface.srv as srvs
import tl_ros2_interface.msg as msgs


def setup_module():
    rclpy.init()


def teardown_module():
    rclpy.shutdown()


def test_service_handlers():
    node = TLArmNode()

    # Trigger services
    req = Trigger.Request()
    resp = Trigger.Response()
    resp = node.handle_connect_service(req, resp)
    assert resp.success

    resp = node.handle_disconnect_service(req, resp)
    assert resp.success

    resp = node.handle_poweron_service(req, resp)
    assert resp.success

    resp = node.handle_poweroff_service(req, resp)
    assert resp.success

    resp = node.handle_clear_error_service(req, resp)
    assert resp.success

    # SetSpeed service
    if hasattr(srvs, "SetSpeed"):
        req2 = srvs.SetSpeed.Request()
        req2.speed = 1.5
        resp2 = srvs.SetSpeed.Response()
        resp2 = node.handle_set_speed_service(req2, resp2)
        assert resp2.success


def test_topic_handlers():
    node = TLArmNode()

    # MoveCommand message
    cmd = msgs.MoveCommand()
    cmd.target_pos_value = [0.0, 0.0, 0.0]
    ok = node.handle_movej_topic(cmd)
    assert ok is True

    job = msgs.JobInsertMove()
    job.line = 1
    job.cmd = cmd
    ok = node.handle_job_insert_movej_topic(job)
    assert ok is True

    ok = node.handle_movel_topic(cmd)
    assert ok is True

    ok = node.handle_job_insert_movel_topic(job)
    assert ok is True


# *** End Patch
