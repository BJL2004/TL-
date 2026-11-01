# import pyrealsense2 as rs

# pipe = rs.pipeline()
# cfg = rs.config()
# cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# pipe.start(cfg)
# for i in range(30):
#     frames = pipe.wait_for_frames()
#     color = frames.get_color_frame()
#     depth = frames.get_depth_frame()
#     if color and depth:
#         print(f'Frame {i}: color={color.get_width()}x{color.get_height()}, depth={depth.get_width()}x{depth.get_height()}')
# pipe.stop()
# print('D435 OK')
import pyrealsense2 as rs
import numpy as np
import cv2

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipe.start(cfg)

align = rs.align(rs.stream.color)

print("D435 画面 | 左=彩色 右=深度 | q 退出")
try:
    while True:
        frames = pipe.wait_for_frames()
        aligned = align.process(frames)
        color = aligned.get_color_frame()
        depth = aligned.get_depth_frame()
        if not color or not depth:
            continue

        color_img = np.asanyarray(color.get_data())
        depth_img = np.asanyarray(depth.get_data())
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_img, alpha=0.03), cv2.COLORMAP_JET
        )

        stacked = np.hstack((color_img, depth_colormap))
        cv2.imshow("D435 | Left=Color Right=Depth", stacked)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipe.stop()
    cv2.destroyAllWindows()