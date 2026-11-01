import pyrealsense2 as rs, numpy as np, rclpy, math
from rclpy.node import Node
from tl_ros2_interface.msg import CartesianPose
from scipy.spatial.transform import Rotation as R

rclpy.init()
n = Node('diag')
tcp = [None]
def cb(m): tcp[0]=m
n.create_subscription(CartesianPose,'/tcp_pose',cb,10)

pipe=rs.pipeline(); cfg=rs.config()
cfg.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
cfg.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
pipe.start(cfg); align=rs.align(rs.stream.color)
prof=pipe.get_active_profile()
intr=prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

for i in range(30):
    rclpy.spin_once(n, timeout_sec=0.1)
    frames=pipe.wait_for_frames()
    if tcp[0] is None: continue
    aligned=align.process(frames)
    c=aligned.get_color_frame(); d=aligned.get_depth_frame()
    if not c or not d: continue
    
    depth=np.asanyarray(d.get_data())
    z_center=np.median(depth[230:250,310:330][depth[230:250,310:330]>0])/1000. if np.any(depth[230:250,310:330]>0) else 0
    cx_px,cy_px=320,240
    cam_x=(cx_px-intr.ppx)*z_center/intr.fx
    cam_y=(cy_px-intr.ppy)*z_center/intr.fy
    cam_z=z_center
    
    tp=tcp[0]
    pos=np.array([tp.position.x/1000.,tp.position.y/1000.,tp.position.z/1000.])
    rpy=np.array([tp.rpy.x,tp.rpy.y,tp.rpy.z])
    print('TCP pos: %.3f %.3f %.3f m' % (pos[0],pos[1],pos[2]))
    print('cam center: %.3f %.3f %.3f m' % (cam_x,cam_y,cam_z))
    
    T_tool_cam=np.array([[-0.013,-0.999,0.023,0.077],[0.999,-0.013,0.019,-0.040],[-0.018,0.023,0.999,0.009],[0,0,0,1]])
    Rt=R.from_euler('XYZ',rpy).as_matrix()
    T_base_tool=np.eye(4); T_base_tool[:3,:3]=Rt; T_base_tool[:3,3]=pos
    pb=T_base_tool@T_tool_cam@np.array([cam_x,cam_y,cam_z,1])
    bx=int(pb[0]*1000); by=int(pb[1]*1000); bz=int(pb[2]*1000)
    print('base(x86): %d %d %d mm' % (bx,by,bz))
    break

pipe.stop(); n.destroy_node(); rclpy.shutdown()
print("DONE")
