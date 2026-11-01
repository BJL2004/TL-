#!/usr/bin/env python3
"""生成灵巧手机械臂项目问题诊断报告 Word文档"""

from docx import Document
from docx.shared import Pt, Cm, RGBColor, Inches
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml.ns import qn
from docx.oxml import OxmlElement
import os

OUTPUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "灵巧手机械臂项目问题诊断报告.docx")

# ── 颜色常量 ──
RED = RGBColor(0xA3, 0x2D, 0x2D)
ORANGE = RGBColor(0x99, 0x3C, 0x1D)
AMBER = RGBColor(0x85, 0x4F, 0x0B)
GREEN = RGBColor(0x0F, 0x6E, 0x56)
GRAY = RGBColor(0x60, 0x60, 0x60)
BLACK = RGBColor(0x1A, 0x1A, 0x1A)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)
CODE_BG = "F0F0F0"

doc = Document()

# ── 全局样式 ──
style = doc.styles['Normal']
font = style.font
font.name = 'Microsoft YaHei'
font.size = Pt(10.5)
style.element.rPr.rFonts.set(qn('w:eastAsia'), 'Microsoft YaHei')
style.paragraph_format.space_after = Pt(4)
style.paragraph_format.line_spacing = 1.25

def set_cell_font(cell, text, size=10, bold=False, color=BLACK, font_name='Microsoft YaHei'):
    cell.text = ''
    p = cell.paragraphs[0]
    run = p.add_run(text)
    run.font.size = Pt(size)
    run.font.bold = bold
    run.font.color.rgb = color
    run.font.name = font_name
    run.element.rPr.rFonts.set(qn('w:eastAsia'), font_name)
    p.paragraph_format.space_after = Pt(2)
    p.paragraph_format.space_before = Pt(2)

def shade_cell(cell, color_hex):
    shading = OxmlElement('w:shd')
    shading.set(qn('w:fill'), color_hex)
    cell._tc.get_or_add_tcPr().append(shading)

def add_heading(text, level=1, color=None):
    h = doc.add_heading(text, level=level)
    for run in h.runs:
        run.font.name = 'Microsoft YaHei'
        run.element.rPr.rFonts.set(qn('w:eastAsia'), 'Microsoft YaHei')
        if color:
            run.font.color.rgb = color
    return h

def add_para(text, bold=False, color=BLACK, size=10.5, indent=None):
    p = doc.add_paragraph()
    run = p.add_run(text)
    run.font.bold = bold
    run.font.color.rgb = color
    run.font.size = Pt(size)
    run.font.name = 'Microsoft YaHei'
    run.element.rPr.rFonts.set(qn('w:eastAsia'), 'Microsoft YaHei')
    if indent:
        p.paragraph_format.left_indent = Cm(indent)
    return p

def add_code(text):
    p = doc.add_paragraph()
    p.paragraph_format.left_indent = Cm(1)
    p.paragraph_format.space_before = Pt(2)
    p.paragraph_format.space_after = Pt(4)
    run = p.add_run(text)
    run.font.name = 'Consolas'
    run.font.size = Pt(9.5)
    run.font.color.rgb = RGBColor(0x33, 0x33, 0x33)
    # 背景色
    pPr = p._p.get_or_add_pPr()
    shd = OxmlElement('w:shd')
    shd.set(qn('w:fill'), CODE_BG)
    pPr.append(shd)
    return p

def add_bullet(text, level=0):
    p = doc.add_paragraph(style='List Bullet')
    if level > 0:
        p.paragraph_format.left_indent = Cm(1.5 + level * 0.8)
    run = p.add_run(text)
    run.font.size = Pt(10)
    run.font.name = 'Microsoft YaHei'
    run.element.rPr.rFonts.set(qn('w:eastAsia'), 'Microsoft YaHei')
    return p

def add_table(headers, rows, col_widths=None, header_color="534AB7"):
    table = doc.add_table(rows=1 + len(rows), cols=len(headers))
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.style = 'Table Grid'
    # 表头
    for i, h in enumerate(headers):
        cell = table.rows[0].cells[i]
        set_cell_font(cell, h, size=9.5, bold=True, color=WHITE)
        shade_cell(cell, header_color)
    # 数据行
    for r_idx, row in enumerate(rows):
        for c_idx, val in enumerate(row):
            cell = table.rows[r_idx + 1].cells[c_idx]
            set_cell_font(cell, str(val), size=9.5)
    # 列宽
    if col_widths:
        for i, w in enumerate(col_widths):
            for row in table.rows:
                row.cells[i].width = Cm(w)
    return table

# ════════════════════════════════════════════════════════════
# 文档内容
# ════════════════════════════════════════════════════════════

# ── 标题页 ──
title = doc.add_paragraph()
title.alignment = WD_ALIGN_PARAGRAPH.CENTER
run = title.add_run('灵巧手机械臂项目\n问题诊断与修复方案')
run.font.size = Pt(24)
run.font.bold = True
run.font.name = 'Microsoft YaHei'
run.element.rPr.rFonts.set(qn('w:eastAsia'), 'Microsoft YaHei')
run.font.color.rgb = RGBColor(0x26, 0x21, 0x5C)
title.paragraph_format.space_before = Pt(60)
title.paragraph_format.space_after = Pt(20)

subtitle = doc.add_paragraph()
subtitle.alignment = WD_ALIGN_PARAGRAPH.CENTER
run = subtitle.add_run('Jetson Orin Nano 16GB + 天链 TCB605 + 傲意 ROH-LiteS001 + Intel RealSense D435')
run.font.size = Pt(11)
run.font.color.rgb = GRAY
run.font.name = 'Microsoft YaHei'
run.element.rPr.rFonts.set(qn('w:eastAsia'), 'Microsoft YaHei')

info = doc.add_paragraph()
info.alignment = WD_ALIGN_PARAGRAPH.CENTER
run = info.add_run('生成日期: 2026-07-23  |  分析范围: 全部工程代码')
run.font.size = Pt(10)
run.font.color.rgb = GRAY
run.font.name = 'Microsoft YaHei'
run.element.rPr.rFonts.set(qn('w:eastAsia'), 'Microsoft YaHei')

doc.add_page_break()

# ════════════════════════════════════════════════════════════
# 一、项目概述
# ════════════════════════════════════════════════════════════
add_heading('一、项目概述', level=1, color=RGBColor(0x26, 0x21, 0x5C))

add_table(
    ['维度', '详情'],
    [
        ['硬件平台', 'Jetson Orin Nano 16GB (40 TOPS) + 天链 TCB605 6轴机械臂 + 傲意 ROH-LiteS001 6指灵巧手 + Intel RealSense D435 深度相机'],
        ['软件栈', 'JetPack 6.2 / CUDA 12.6 / TensorRT 10.3 / ROS2 Humble / PyTorch 2.8 GPU / YOLOv8n / pyrealsense2'],
        ['代码规模', '489 个 Python 文件、34 个 Markdown 文档、40 个固件更新包'],
        ['核心目录', 'hand_demo_arm (Jetson主程序) / tl_robot_ros2_py-arm (ROS2驱动) / tl_vision (视觉+手眼标定) / roh_firmware (灵巧手固件)'],
        ['通信架构', '机械臂 TCP(192.168.1.13) + 灵巧手 ModBus-RTU(RS485端口2, 115200) + D435 USB'],
        ['当前状态', '底层驱动100%完成；视觉抓取因坐标变换错误无法正常工作'],
    ],
    col_widths=[3, 13]
)

doc.add_paragraph()

# ════════════════════════════════════════════════════════════
# 二、问题总览
# ════════════════════════════════════════════════════════════
add_heading('二、问题总览', level=1, color=RGBColor(0x26, 0x21, 0x5C))

add_para('以下问题按严重程度排序，红色为导致视觉抓取完全失败的根因。', color=GRAY, size=10)

add_table(
    ['#', '严重程度', '问题', '影响范围', '修复状态'],
    [
        ['1', '严重-根因', '欧拉角约定不一致\n(标定extrinsic xyz vs 运行时intrinsic XYZ)', '坐标变换完全错误\n机械臂抓不到物体', '已提供诊断工具'],
        ['2', '阻塞', '手眼标定矩阵来自x86 VM\n(Jetson上D435安装角度不同)', '标定矩阵物理上不匹配\n抓取位置偏差几十厘米', '需重新标定'],
        ['3', '严重', '标定采集格式不兼容\n(空格分隔 vs 逗号分隔)', 'calibrate_collect.py\n输出无法被标定脚本读取', '已修复'],
        ['4', '严重', 'arm_hand_demo.py\nstart_addr=False', '该脚本中灵巧手完全不响应', '待修复'],
        ['5', '中等', 'control_node.yaml\nhand_open配置不一致', '自动循环脚本\n后三指手势行为异常', '待修复'],
        ['6', '性能', 'YOLO推理使用CPU\n(vision_grasp_demo.py:313)', '检测速度慢\n拖慢抓取节拍', '待修复'],
        ['7', '低', '约240MB冗余压缩包\n+ 重复目录/文件', '磁盘空间浪费', '建议清理'],
    ],
    col_widths=[1, 2.5, 5, 4, 3],
    header_color="A32D2D"
)

doc.add_page_break()

# ════════════════════════════════════════════════════════════
# 三、详细问题分析
# ════════════════════════════════════════════════════════════
add_heading('三、详细问题分析', level=1, color=RGBColor(0x26, 0x21, 0x5C))

# ── 问题1: 欧拉角约定 ──
add_heading('问题 1  欧拉角约定不一致（严重 — 根本原因）', level=2, color=RED)

add_para('现象', bold=True, color=ORANGE, size=11)
add_para('视觉抓取完全失败，机械臂走到与目标物体毫不相关的位置。即使用 vision_grasp_demo.py 的 [t] 测试偏移功能，末端也偏离目标很远。')

add_para('根因分析', bold=True, color=ORANGE, size=11)
add_para('标定脚本和运行时代码使用了不同的欧拉角旋转约定。标定时用一种约定计算手眼矩阵，运行时用另一种约定构建旋转矩阵，导致 T_base_tool 的旋转部分与标定时不一致，进而 T_base_cam 完全偏离。')

add_para('涉及文件对比', bold=True, color=ORANGE, size=11)

add_table(
    ['位置', '代码', '约定', '说明'],
    [
        ['save_poses.py:28', 'R = Rz @ Ry @ Rx', 'extrinsic xyz (固定轴)', '标定脚本，注释明确"先绕x轴 再绕y轴 再绕z轴"'],
        ['save_poses2.py:47', 'R = Rz @ Ry @ Rx', 'extrinsic xyz (固定轴)', '眼在手外标定，同上'],
        ['vision_grasp_demo.py:343', "R.from_euler('XYZ', rpy)", 'intrinsic XYZ (运动轴)', 'ARM版视觉抓取主程序'],
        ['control_node.py:564', "R.from_euler('XYZ', [A,B,C])", 'intrinsic XYZ (运动轴)', 'ARM版自动循环控制'],
        ['hand_demo/control_node.py:661', "R.from_euler('XYZ', ...)", 'intrinsic XYZ (运动轴)', 'x86版(同样有此问题)'],
    ],
    col_widths=[5, 4, 3.5, 4.5]
)

doc.add_paragraph()
add_para('数学差异', bold=True, color=ORANGE, size=11)
add_para('矩阵乘法不可交换，两种约定对同一组 RPY 角度产生不同的旋转矩阵：')
add_code("extrinsic xyz = Rz(rz) @ Ry(ry) @ Rx(rx) = R.from_euler('xyz', [rx,ry,rz])\n"
         "intrinsic XYZ = Rx(rx) @ Ry(ry) @ Rz(rz) = R.from_euler('XYZ', [rx,ry,rz])\n\n"
         "Rz @ Ry @ Rx  !=  Rx @ Ry @ Rz   (除非角度退化)")

add_para('影响链条', bold=True, color=ORANGE, size=11)
add_bullet('标定时: save_poses.py 用 Rz@Ry@Rx 构建 T_gripper2base → cv2.calibrateHandEye 求解 T_cam2gripper')
add_bullet('运行时: 代码用 Rx@Ry@Rz 构建 T_base_tool → 旋转矩阵与标定时不一致')
add_bullet('结果: T_base_cam = T_base_tool(错) @ T_tool_camera → 基座坐标完全偏离 → 抓取失败')

add_para('修复方案', bold=True, color=GREEN, size=11)
add_para('将运行时代码的欧拉角约定改为与标定脚本一致（需先用诊断工具确认天链 TCB605 的实际 RPY 约定）：')
add_code("# vision_grasp_demo.py 第343行\n"
         "R_t = R.from_euler('XYZ', rpy).as_matrix()    # 修改前\n"
         "R_t = R.from_euler('xyz', rpy).as_matrix()    # 修改后 (小写=extrinsic)\n\n"
         "# control_node.py 第564行\n"
         "rotation = R.from_euler('XYZ', [A, B, C], degrees=False)    # 修改前\n"
         "rotation = R.from_euler('xyz', [A, B, C], degrees=False)    # 修改后")

doc.add_page_break()

# ── 问题2: 手眼标定矩阵 ──
add_heading('问题 2  手眼标定矩阵来自 x86 VM（阻塞）', level=2, color=RED)

add_para('现象', bold=True, color=ORANGE, size=11)
add_para('即使修复欧拉角约定，抓取位置仍有系统性偏移。')

add_para('根因', bold=True, color=ORANGE, size=11)
add_para('当前使用的手眼矩阵 T_tool_camera 是在 x86 虚拟机上标定的。x86 VM 上 D435 安装在机械臂末端的角度和位置与 Jetson 实体机不完全一致。手眼矩阵是 4x4 齐次变换矩阵，微小的方向偏差会导致末端几十厘米的偏移。')

add_para('当前手眼矩阵', bold=True, color=ORANGE, size=11)
add_code("T_tool_camera =\n"
         "[ 0.0584  -0.9980  -0.0228   0.0409 ]   (米)\n"
         "[ 0.9983   0.0586  -0.0066  -0.0040 ]\n"
         "[ 0.0079  -0.0224   0.9997  -0.0696 ]\n"
         "[ 0.0000   0.0000   0.0000   1.0000 ]\n\n"
         "旋转: ~90度绕Z轴 (相机相对工具坐标系)\n"
         "平移: X=41mm Y=-4mm Z=-70mm")

add_para('修复方案', bold=True, color=GREEN, size=11)
add_bullet('打印 9x6 棋盘格 (格子 25mm)，固定在桌面')
add_bullet('用 calibrate_collect.py 采集 15-20 组不同角度的图像+TCP位姿 (已修复格式bug)')
add_bullet('将采集数据拷入 tl_vision/hand_eye_calibration-main-1/eye_hand_data/')
add_bullet('运行 compute_in_hand.py 求解 T_tool_camera')
add_bullet('将结果填入 vision_grasp_demo.py 第54-58行 或 control_node.yaml 第47-53行')

doc.add_paragraph()

# ── 问题3: 标定采集格式 ──
add_heading('问题 3  标定采集格式不兼容（严重 — 已修复）', level=2, color=ORANGE)

add_para('现象', bold=True, color=ORANGE, size=11)
add_para('calibrate_collect.py 采集的位姿文件无法被 save_poses.py 正确读取，导致标定流程中断。')

add_para('根因', bold=True, color=ORANGE, size=11)
add_table(
    ['文件', '写入格式', '读取格式', '结果'],
    [
        ['calibrate_collect.py:88', '空格分隔', '—', '生成 pose_xxx.txt'],
        ['save_poses.py:70', '—', '逗号分隔 split(",")', '解析失败'],
    ],
    col_widths=[5, 3, 3, 4]
)
doc.add_paragraph()
add_code("# calibrate_collect.py 修改前 (空格分隔)\n"
         "f.write(f'{x} {y} {z} {rx} {ry} {rz}\\n')\n\n"
         "# calibrate_collect.py 修改后 (逗号分隔, 与save_poses.py匹配)\n"
         "f.write(f'{x},{y},{z},{rx},{ry},{rz}\\n')")

add_para('修复状态: 已完成', bold=True, color=GREEN, size=11)

doc.add_paragraph()

# ── 问题4: start_addr ──
add_heading('问题 4  arm_hand_demo.py 灵巧手 start_addr 配置错误（严重）', level=2, color=ORANGE)

add_para('现象', bold=True, color=ORANGE, size=11)
add_para('使用 arm_hand_demo.py 时灵巧手完全不响应，ModBus 写入返回成功但手指不动。')

add_para('根因', bold=True, color=ORANGE, size=11)
add_para('arm_hand_demo.py 第251行将 start_addr 设为 False，但文档和其他所有文件均明确 start_addr 必须为 True（PLC 寻址模式）。False 模式下寄存器地址偏移，写入的位置值实际落到了错误的地址。')

add_table(
    ['文件', 'start_addr', '状态'],
    [
        ['arm_hand_demo.py:251', 'False', '错误!'],
        ['vision_grasp_demo.py:250', 'True', '正确'],
        ['control_node.py:794', 'True', '正确'],
        ['hand_controller.py:241', 'True', '正确'],
    ],
    col_widths=[7, 3, 4]
)

add_para('修复方案', bold=True, color=GREEN, size=11)
add_code("# arm_hand_demo.py 第251行\n"
         "param.start_addr = False    # 修改前\n"
         "param.start_addr = True     # 修改后")

doc.add_paragraph()

# ── 问题5: hand_open配置 ──
add_heading('问题 5  control_node.yaml 手势配置不一致（中等）', level=2, color=AMBER)

add_para('现象', bold=True, color=ORANGE, size=11)
add_para('control_node.py (走 yaml 配置) 的灵巧手"张开"行为与 vision_grasp_demo.py / arm_hand_demo.py (代码内硬编码) 不一致。')

add_para('根因', bold=True, color=ORANGE, size=11)
add_para('control_node.yaml 中 modbus_gripper_open_val 的后三指值为 60000 (实际是闭合值)，而代码内的 hand_open 全部为 [0,0,0,0,0,0] (全部张开)。')

add_table(
    ['来源', 'hand_open 值', '后三指含义'],
    [
        ['control_node.yaml:22', '[0, 0, 0, 60000, 60000, 60000]', '后三指=60000 (闭合!)'],
        ['vision_grasp_demo.py:59', '[0, 0, 0, 0, 0, 0]', '全部张开'],
        ['arm_hand_demo.py:46', '[0, 0, 0, 0, 0, 0]', '全部张开'],
        ['hand_controller.py:80 (Gesture.OPEN)', '[0, 0, 0, 0, 0, 0]', '全部张开'],
    ],
    col_widths=[6, 5, 4]
)

add_para('修复方案', bold=True, color=GREEN, size=11)
add_code("# control_node.yaml 第22行\n"
         "modbus_gripper_open_val: [0, 0, 0, 60000, 60000, 60000]    # 修改前\n"
         "modbus_gripper_open_val: [0, 0, 0, 0, 0, 0]                # 修改后")

doc.add_paragraph()

# ── 问题6: YOLO CPU ──
add_heading('问题 6  YOLO 推理使用 CPU（性能）', level=2, color=AMBER)

add_para('现象', bold=True, color=ORANGE, size=11)
add_para('文档称 YOLOv8n 在 Jetson 上用 GPU 推理可达 20-30 FPS，但实际检测帧率明显偏低。')

add_para('根因', bold=True, color=ORANGE, size=11)
add_para('vision_grasp_demo.py 第313行硬编码 device=\'cpu\'，未使用 Jetson GPU。')
add_code("# vision_grasp_demo.py 第313行\n"
         "results = self.model(color_np, conf=0.5, device='cpu', verbose=False)     # 修改前\n"
         "results = self.model(color_np, conf=0.5, device=0, verbose=False)        # 修改后 (GPU)")

doc.add_paragraph()

# ── 问题7: 冗余文件 ──
add_heading('问题 7  冗余文件（低）', level=2, color=GRAY)

add_table(
    ['冗余项', '大小', '说明'],
    [
        ['roh_firmware.zip', '98 MB', '已解压为同名目录'],
        ['tl_vision.zip', '72 MB', '已解压为同名目录'],
        ['tl_robot_ros2_py-arm.zip', '36 MB', '已解压为同名目录'],
        ['tl_robot_ros2_py-x86.zip', '36 MB', '已解压为同名目录'],
        ['hand_demo/ (x86旧版)', '—', '与 hand_demo_arm/ 并存，如不再维护可归档'],
        ['README - 副本.md', '—', '与 README.md 重复'],
    ],
    col_widths=[6, 2.5, 7]
)
doc.add_paragraph()
add_para('建议: 确认不再需要后清理上述压缩包，可释放约 240MB 空间。', color=GRAY, size=10)

doc.add_page_break()

# ════════════════════════════════════════════════════════════
# 四、修复优先级与行动计划
# ════════════════════════════════════════════════════════════
add_heading('四、修复优先级与行动计划', level=1, color=RGBColor(0x26, 0x21, 0x5C))

add_table(
    ['优先级', '行动', '预计工作量', '前置条件'],
    [
        ['P0', '运行 diag_transform.py 确认天链RPY约定\n(extrinsic xyz 还是 intrinsic XYZ)', '10分钟', '机械臂+D435就绪'],
        ['P1', '根据诊断结果统一欧拉角约定\n(vision_grasp_demo.py + control_node.py)', '5分钟', 'P0完成'],
        ['P1', '修复 arm_hand_demo.py start_addr\n(False → True)', '1分钟', '无'],
        ['P1', '修复 control_node.yaml hand_open\n(后三指 60000 → 0)', '1分钟', '无'],
        ['P2', '在 Jetson 上执行手眼标定\n(棋盘格 + calibrate_collect.py + compute_in_hand.py)', '1-2小时', 'P1完成'],
        ['P2', '用 vision_grasp_demo.py [t] 验证抓取精度', '15分钟', 'P2标定完成'],
        ['P3', 'YOLO 切换 GPU 推理\n(device=\'cpu\' → device=0)', '1分钟', '无'],
        ['P3', '清理冗余压缩包', '5分钟', '确认无需保留'],
    ],
    col_widths=[1.5, 7, 2.5, 4],
    header_color="534AB7"
)

doc.add_paragraph()
add_para('关键路径: P0 → P1(欧拉角) → P2(标定) → P2(验证)', bold=True, color=GREEN, size=11)

doc.add_page_break()

# ════════════════════════════════════════════════════════════
# 五、诊断工具使用说明
# ════════════════════════════════════════════════════════════
add_heading('五、诊断工具使用说明', level=1, color=RGBColor(0x26, 0x21, 0x5C))

add_para('已创建诊断脚本 hand_demo_arm/diag_transform.py，用于确定正确的欧拉角约定。', size=10.5)

add_para('启动方式', bold=True, color=ORANGE, size=11)
add_code("# 终端1: 启动机械臂驱动\n"
         "source ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash\n"
         "ros2 launch tl_driver tl_driver.launch.py arm_type:=tcb605\n\n"
         "# 终端2: 运行诊断工具\n"
         "cd ~/Desktop/hand_demo_arm\n"
         "python3 diag_transform.py")

add_para('操作流程', bold=True, color=ORANGE, size=11)
add_bullet('把一个物体放在机械臂前方桌面上 (记住大致位置)')
add_bullet('按 [d] 检测 — 脚本会同时用两种约定计算基座坐标并打印对比')
add_bullet('观察输出: 哪种约定算出的坐标更接近物体真实位置')
add_bullet('按 [t] 让机械臂移到 extrinsic xyz 算出的位置上方 — 肉眼观察偏移')
add_bullet('按 [T] 让机械臂移到 intrinsic XYZ 算出的位置上方 — 肉眼观察偏移')
add_bullet('偏移更小的那种约定就是正确的')

add_para('输出示例', bold=True, color=ORANGE, size=11)
add_code("============================================================\n"
         "  检测到: bottle\n"
         "  相机坐标: X=0.123 Y=-0.045 Z=0.567 (米)\n"
         "  TCP 位姿: x=350.0 y=120.0 z=280.0 (mm)\n"
         "  TCP RPY:  r=3.1300 p=0.9200 y=0.0000 (rad)\n"
         "------------------------------------------------------------\n"
         "  约定A extrinsic xyz (标定脚本同款):\n"
         "    基座坐标: X=420.5  Y=115.3  Z=180.2  (mm)\n"
         "  约定B intrinsic XYZ  (当前代码同款):\n"
         "    基座坐标: X=158.7  Y=-230.4  Z=95.1  (mm)\n"
         "------------------------------------------------------------\n"
         "  两者偏差: 385.2 mm\n"
         "  → 看哪个坐标更接近物体真实位置，就用那个约定。\n"
         "============================================================")

add_para('判断标准', bold=True, color=GREEN, size=11)
add_bullet('如果物体实际在 X≈420 Y≈115 位置 → 约定A (extrinsic xyz) 正确 → 运行时代码改为 R.from_euler(\'xyz\', rpy)')
add_bullet('如果物体实际在 X≈159 Y≈-230 位置 → 约定B (intrinsic XYZ) 正确 → 标定脚本需改用 R.from_euler(\'XYZ\')')
add_bullet('将诊断结果告知，可一次性修复全部相关代码')

# ════════════════════════════════════════════════════════════
# 六、附录
# ════════════════════════════════════════════════════════════
doc.add_page_break()
add_heading('六、附录：关键参数速查', level=1, color=RGBColor(0x26, 0x21, 0x5C))

add_table(
    ['参数', '值', '位置'],
    [
        ['机械臂 IP', '192.168.1.13', 'tl_driver/config/tcb605_config.yaml'],
        ['机械臂型号', 'TCB605 (6轴)', 'launch 文件 arm_type 参数'],
        ['灵巧手 slave_id', '2', 'ModBus RTU'],
        ['灵巧手端口', '2 (RS485)', '机械臂末端航插 9-10针'],
        ['灵巧手波特率', '115200', '—'],
        ['灵巧手 start_addr', 'True (必须!)', 'PLC寻址模式'],
        ['灵巧手初始化', 'SELF_TEST_LEVEL=1 → START_INIT=1', '寄存器 1008 → 1013'],
        ['位置目标寄存器', '1135-1140 (6指)', '0=张开, 65535=闭合'],
        ['D435 分辨率', '640x480 @ 30fps', '彩色+深度双流'],
        ['YOLO 模型', 'yolov8n.pt (~6MB)', '~/.config/Ultralytics/'],
        ['手眼标定矩阵', 'x86 VM (需重新标定!)', 'control_node.yaml / vision_grasp_demo.py'],
        ['抓取姿态 RPY', 'R=3.13 P=0.92 Y=0.0 (rad)', '硬编码, x86沿用'],
        ['ServoJ 插值', '100-500点, 10Hz', 'vmax=10 amax=3000 jmax=50000'],
    ],
    col_widths=[4, 6, 5]
)

doc.add_paragraph()
add_para('— 报告结束 —', color=GRAY, size=10)
p = doc.paragraphs[-1]
p.alignment = WD_ALIGN_PARAGRAPH.CENTER

# 保存
doc.save(OUTPUT)
print(f"文档已生成: {OUTPUT}")
print(f"文件大小: {os.path.getsize(OUTPUT)/1024:.1f} KB")
