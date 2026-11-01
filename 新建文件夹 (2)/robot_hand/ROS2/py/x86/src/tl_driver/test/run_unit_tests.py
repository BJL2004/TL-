import sys
import rclpy
from tl_driver.tl_driver.tl_driver_node import TLArmNode
from std_srvs.srv import Trigger
import tl_ros2_interface.srv as srvs
import tl_ros2_interface.msg as msgs


def main():
    rclpy.init()
    failures = 0
    try:
        node = TLArmNode()

        # Trigger services
        req = Trigger.Request()
        resp = Trigger.Response()
        resp = node.handle_connect_service(req, resp)
        failures += 0 if resp.success else 1

        resp = node.handle_disconnect_service(req, resp)
        failures += 0 if resp.success else 1

        resp = node.handle_poweron_service(req, resp)
        failures += 0 if resp.success else 1

        resp = node.handle_poweroff_service(req, resp)
        failures += 0 if resp.success else 1

        resp = node.handle_clear_error_service(req, resp)
        failures += 0 if resp.success else 1

        # SetSpeed
        if hasattr(srvs, "SetSpeed"):
            req2 = srvs.SetSpeed.Request()
            req2.speed = 1.5
            resp2 = srvs.SetSpeed.Response()
            resp2 = node.handle_set_speed_service(req2, resp2)
            failures += 0 if resp2.success else 1

        # Topic handlers
        cmd = msgs.MoveCommand()
        cmd.target_pos_value = [0.0, 0.0, 0.0]
        ok = node.handle_movej_topic(cmd)
        failures += 0 if ok else 1

        job = msgs.JobInsertMove()
        job.line = 1
        job.cmd = cmd
        ok = node.handle_job_insert_movej_topic(job)
        failures += 0 if ok else 1

        ok = node.handle_movel_topic(cmd)
        failures += 0 if ok else 1

        ok = node.handle_job_insert_movel_topic(job)
        failures += 0 if ok else 1

    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass

    if failures == 0:
        print("ALL TESTS PASSED")
        sys.exit(0)
    else:
        print(f"{failures} TEST(S) FAILED")
        sys.exit(2)


if __name__ == "__main__":
    main()
