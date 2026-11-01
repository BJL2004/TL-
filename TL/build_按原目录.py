from io import BytesIO
from pathlib import Path
from zipfile import ZipFile

from docx import Document
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT, WD_TABLE_ALIGNMENT
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches, Pt, RGBColor


ROOT = Path(r"C:\Users\DragonAfterAll\Desktop\TL")
SOURCE = ROOT / "说明文档.docx"
OUTPUT = ROOT / "说明文档_完善版.docx"
BLUE, DARK, TEXT, MUTED = "2E74B5", "1F4D78", "1F2937", "5B6470"
FILL, WARN, RISK = "E8EEF5", "FFF4D6", "FCE8E6"


def font(run, size=10.5, color=TEXT, bold=None, name="Calibri"):
    run.font.name = name
    run._element.get_or_add_rPr().rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
    run.font.size = Pt(size)
    run.font.color.rgb = RGBColor.from_string(color)
    if bold is not None:
        run.bold = bold


def margins(cell):
    tcpr = cell._tc.get_or_add_tcPr()
    tc_mar = tcpr.find(qn("w:tcMar"))
    if tc_mar is None:
        tc_mar = OxmlElement("w:tcMar")
        tcpr.append(tc_mar)
    for side, value in (("top", 80), ("start", 120), ("bottom", 80), ("end", 120)):
        node = tc_mar.find(qn(f"w:{side}"))
        if node is None:
            node = OxmlElement(f"w:{side}")
            tc_mar.append(node)
        node.set(qn("w:w"), str(value))
        node.set(qn("w:type"), "dxa")


def table_geometry(table, widths):
    table.alignment = WD_TABLE_ALIGNMENT.LEFT
    table.autofit = False
    pr = table._tbl.tblPr
    layout = pr.find(qn("w:tblLayout"))
    if layout is None:
        layout = OxmlElement("w:tblLayout")
        pr.append(layout)
    layout.set(qn("w:type"), "fixed")
    width = pr.find(qn("w:tblW"))
    if width is None:
        width = OxmlElement("w:tblW")
        pr.append(width)
    width.set(qn("w:w"), str(sum(widths)))
    width.set(qn("w:type"), "dxa")
    indent = pr.find(qn("w:tblInd"))
    if indent is None:
        indent = OxmlElement("w:tblInd")
        pr.append(indent)
    indent.set(qn("w:w"), "120")
    indent.set(qn("w:type"), "dxa")
    grid = table._tbl.tblGrid
    for item in list(grid):
        grid.remove(item)
    for value in widths:
        col = OxmlElement("w:gridCol")
        col.set(qn("w:w"), str(value))
        grid.append(col)
    for row in table.rows:
        for idx, cell in enumerate(row.cells):
            cell.width = Inches(widths[idx] / 1440)
            cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
            margins(cell)


def cell_text(cell, value, bold=False, color=TEXT, size=9.5):
    cell.text = ""
    p = cell.paragraphs[0]
    p.paragraph_format.space_after = Pt(0)
    p.paragraph_format.line_spacing = 1.1
    r = p.add_run(str(value))
    font(r, size=size, color=color, bold=bold)


def add_table(doc, headers, rows, widths):
    table = doc.add_table(rows=1, cols=len(headers))
    table_geometry(table, widths)
    header = table.rows[0]
    trpr = header._tr.get_or_add_trPr()
    trh = OxmlElement("w:tblHeader")
    trh.set(qn("w:val"), "true")
    trpr.append(trh)
    for idx, value in enumerate(headers):
        cell_text(header.cells[idx], value, True, DARK)
        shd = OxmlElement("w:shd")
        shd.set(qn("w:fill"), FILL)
        header.cells[idx]._tc.get_or_add_tcPr().append(shd)
    for row in rows:
        cells = table.add_row().cells
        for idx, value in enumerate(row):
            cell_text(cells[idx], value)
    doc.add_paragraph().paragraph_format.space_after = Pt(0)


def para(doc, text, style="Normal"):
    p = doc.add_paragraph(style=style)
    p.add_run(text)
    return p


def bullet(doc, text):
    return para(doc, text, "List Bullet")


def number(doc, text):
    return para(doc, text, "List Number")


def code(doc, text):
    return para(doc, text, "Code Block")


def callout(doc, label, text, fill=FILL, color=DARK):
    table = doc.add_table(rows=1, cols=1)
    table_geometry(table, [9360])
    cell = table.cell(0, 0)
    shd = OxmlElement("w:shd")
    shd.set(qn("w:fill"), fill)
    cell._tc.get_or_add_tcPr().append(shd)
    p = cell.paragraphs[0]
    p.paragraph_format.space_after = Pt(0)
    r = p.add_run(label + "：")
    font(r, bold=True, color=color)
    r = p.add_run(text)
    font(r)
    doc.add_paragraph().paragraph_format.space_after = Pt(0)


def style_doc(doc):
    normal = doc.styles["Normal"]
    normal.font.name = "Calibri"
    normal._element.get_or_add_rPr().rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
    normal.font.size = Pt(10.5)
    normal.font.color.rgb = RGBColor.from_string(TEXT)
    normal.paragraph_format.space_after = Pt(6)
    normal.paragraph_format.line_spacing = 1.25
    for name in ("List Bullet", "List Number"):
        st = doc.styles[name]
        st.font.name = "Calibri"
        st._element.get_or_add_rPr().rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
        st.font.size = Pt(10.5)
        st.font.color.rgb = RGBColor.from_string(TEXT)
        st.paragraph_format.left_indent = Inches(0.375)
        st.paragraph_format.first_line_indent = Inches(-0.188)
        st.paragraph_format.space_after = Pt(4)
        st.paragraph_format.line_spacing = 1.2
    code_style = doc.styles.add_style("Code Block", 1)
    code_style.font.name = "Consolas"
    code_style._element.get_or_add_rPr().rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
    code_style.font.size = Pt(8.5)
    code_style.font.color.rgb = RGBColor.from_string("234E70")
    code_style.paragraph_format.left_indent = Inches(0.2)
    code_style.paragraph_format.space_after = Pt(6)
    code_style.paragraph_format.line_spacing = 1.05
    for level, size, color, before, after in ((1, 16, BLUE, 18, 10), (2, 13, BLUE, 14, 7), (3, 11.5, DARK, 10, 5)):
        st = doc.styles[f"Heading {level}"]
        st.font.name = "Calibri"
        st._element.get_or_add_rPr().rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
        st.font.size = Pt(size)
        st.font.bold = True
        st.font.color.rgb = RGBColor.from_string(color)
        st.paragraph_format.space_before = Pt(before)
        st.paragraph_format.space_after = Pt(after)
        st.paragraph_format.keep_with_next = True


def page_setup(doc):
    for section in doc.sections:
        section.page_width = Inches(8.27)
        section.page_height = Inches(11.69)
        section.top_margin = Inches(0.78)
        section.bottom_margin = Inches(0.72)
        section.left_margin = Inches(0.82)
        section.right_margin = Inches(0.82)
        section.header_distance = Inches(0.35)
        section.footer_distance = Inches(0.35)
        hp = section.header.paragraphs[0]
        hp.text = "TL 机械臂 · 灵巧手 · U 型叉系统"
        hp.alignment = WD_ALIGN_PARAGRAPH.RIGHT
        for r in hp.runs:
            font(r, 8.5, MUTED)
        fp = section.footer.paragraphs[0]
        fp.alignment = WD_ALIGN_PARAGRAPH.CENTER
        r = fp.add_run("内部工程说明 · 版本 1.0 · 2026-08-25    第 ")
        font(r, 8, MUTED)
        run = fp.add_run()
        begin = OxmlElement("w:fldChar")
        begin.set(qn("w:fldCharType"), "begin")
        instr = OxmlElement("w:instrText")
        instr.set(qn("xml:space"), "preserve")
        instr.text = " PAGE "
        end = OxmlElement("w:fldChar")
        end.set(qn("w:fldCharType"), "end")
        run._r.extend([begin, instr, end])
        r = fp.add_run(" 页")
        font(r, 8, MUTED)


def headings(doc):
    return [doc.add_heading(value, level=1) for value in [
        "天链机械臂示教上位机软件",
        "手眼标定程序说明",
        "灵巧手使用程序说明",
        "机械臂抓取放置种子示教说明",
        "机械臂叉取花篮剪根程序说明",
        "VLA部署",
        "安全注意事项",
    ]]


def build():
    source_doc = Document(str(SOURCE))
    image_bytes = None
    if source_doc.inline_shapes:
        blip = source_doc.inline_shapes[0]._inline.graphic.graphicData.pic.blipFill.blip
        image_bytes = source_doc.part.related_parts[blip.embed].blob
    doc = Document()
    style_doc(doc)
    page_setup(doc)
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    if image_bytes:
        p.add_run().add_picture(BytesIO(image_bytes), width=Inches(5.25), height=Inches(5.25))
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = p.add_run("TL机械臂灵巧手说明文档")
    font(r, 25, "0B2545", True)
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = p.add_run("天链 TCB605 · Jetson Orin NX 16 GB · D435 · ROS 2 Humble")
    font(r, 11.5, MUTED)
    doc.add_page_break()
    para(doc, "本文严格按照原始说明文档的目录补充。原始七个目录标题及顺序保持不变；正文内容根据当前 TL 工程、README、程序说明和两个子工程的实际文件整理。")

    add_heading = lambda text, level=2: doc.add_heading(text, level=level)
    h = headings(doc)

    add_heading("功能定位与工程组成")
    para(doc, "系统运行在 Jetson Orin NX 16 GB 开发板上，控制天链 TCB605 六轴机械臂。D435 安装在末端，用于深度测量和 ArUco 花篮定位；ROH-LiteS001 灵巧手负责种子抓取/释放；U 型叉子负责叉取花篮并转运到修剪位。")
    add_table(doc, ["模块", "职责", "工程位置"], [
        ("天链驱动", "机械臂连接、TCP/关节状态、ServoJ、IO 和坐标变换", "~/Desktop/tl_robot_ros2_py-arm"),
        ("灵巧手程序", "MQTT 触发取种、目标花篮定位、放种", "TL_Handback/src/tl_hand"),
        ("叉子程序", "U 型叉子示教、携篮转运、放回", "TL_Handback/src/tl_fork_teach"),
        ("种子示教", "记录每个花篮的搜索/接近/释放位", "TL_Dobot/src/tl_hand_deploy"),
    ], [2200, 4300, 2860])
    add_heading("上位机/驱动启动", 2)
    code(doc, "cd ~/Desktop/TL\nsource /opt/ros/humble/setup.bash\nsource ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash\nbash scripts/run_driver.sh")
    para(doc, "天链示教器修改控制器 IP 后，必须同步修改厂商驱动配置并重启驱动。上层程序只依赖 ROS2 服务和话题，不应直接混用另一套控制程序。当前精细定位以 100 Hz ServoJ 为主，MoveJ 仅作为单独对比程序。")
    callout(doc, "启动原则", "驱动必须先启动；种子节点和叉篮节点不能同时运行；任何时候只能有一个运动节点向机械臂发布 ServoJ/MoveJ。", WARN, "7A5A00")

    add_heading("眼在手上坐标链")
    para(doc, "D435 为 eye-in-hand 安装。程序通过 T_base_tool0、T_tool0_camera 和相机反投影把图像点变换到机械臂基座：")
    callout(doc, "公式", "p_base = T_base_tool0 × T_tool0_camera × p_camera。T_base_tool0 来自机械臂实时 TCP，T_tool0_camera 来自手眼标定，p_camera 来自 D435 内参和深度。")
    add_heading("采集步骤", 2)
    for text in [
        "不安装灵巧手也可以采集，但相机支架必须固定，棋盘格必须固定不动。当前棋盘格为 10×7 内角点、15 mm 方格。",
        "启动 handeye_collect.launch.py，手动拖动机械臂；每到一个姿态静止 1~3 秒后按 S 保存。",
        "建议采集 25~35 组，覆盖图像中心和边角、0.3~0.6 m 距离，并绕两个正交方向形成明显姿态变化。",
    ]:
        number(doc, text)
    code(doc, "cd ~/Desktop/TL/TL_Dobot\nsource /opt/ros/humble/setup.bash\nsource ~/Desktop/tl_robot_ros2_py-arm/install/setup.bash\ncolcon build --symlink-install --packages-select tl_hand_deploy\nsource install/setup.bash\nros2 launch tl_hand_deploy handeye_collect.launch.py")
    add_heading("求解、验收和九点验证", 2)
    code(doc, "DATASET=~/tl_handeye_data/session_实际时间\nros2 run tl_hand_deploy tl_handeye_calibrate \"$DATASET\"\ncp \"$DATASET/calibration/handeye_accepted.yaml\" ~/tl_handeye_data/best/handeye_best.yaml")
    add_table(doc, ["误差模式", "结论", "处理"], [
        ("方向和大小基本一致", "固定偏移", "九点验证后加一次 XY 常量补偿"),
        ("随工作区位置变化", "手眼旋转/支架/坐标系问题", "重采集、重标定，不继续叠加均值"),
        ("随机散开", "深度、点击、抖动或离群样本", "增加静止时间和样本质量"),
    ], [2600, 3100, 3660])
    callout(doc, "正式使用标准", "正式自动抓取使用 accepted: true 的标定。candidate_rejected.yaml 只能粗测。安装灵巧手后，若相机支架或工具关系变化，应重标定或至少用掌心 TCP 重新做九点验证。", WARN, "7A5A00")

    add_heading("系统职责")
    para(doc, "灵巧手程序通过 MQTT 收到 seed/seed_place 命令后，从固定取种位置抓取种子，再定位目标花篮并放置。当前正式流程不依赖 YOLO 自动识别种子，取种点由固定参数配置。")
    add_table(doc, ["状态", "动作", "手部策略"], [
        ("待机", "机械臂处于观测位", "力度 1 保护握持"),
        ("取种", "到固定取种位并下降", "力度 5 抓稳种子"),
        ("放置", "到目标花篮接近/释放位", "张开放种"),
        ("完成/失败", "抬升并回观测位", "恢复力度 1"),
    ], [1900, 4800, 2660])
    add_heading("灵巧手保护", 2)
    for text in [
        "真实灵巧手首轮测试先使用低速度、低力度和较大接近间隙，确认掌心/指尖不会压碰物体。",
        "抓取失败、放置完成或通信异常时，优先回到力度 1 的保护状态。",
        "不得为了绕过检查直接把手部限制改成已验证；真实手的张开、闭合、速度、电流和力限制必须空载核对。",
    ]:
        bullet(doc, text)
    add_heading("运行入口", 2)
    code(doc, "cd ~/Desktop/TL\nbash scripts/run_driver.sh          # 终端 A\nbash scripts/run_seed_mqtt.sh       # 终端 B，有画面\n# 或：bash scripts/run_seed_mqtt_headless.sh")

    add_heading("种子示教流程")
    para(doc, "种子示教记录每个花篮的 search_pose、basket_approach 和 basket_release。运行时先到搜索位识别 ArUco，再用当前码位姿补偿花篮的小范围移动。")
    add_heading("示教操作", 2)
    for text in [
        "启动 teach_seed.sh，确认 D435 画面和机械臂处于默认起始位。",
        "拖动到目标花篮，使码完整、稳定、无严重反光；输入编号并按 B 保存搜索位。",
        "拖到花篮上方按 A 保存接近位，再拖到种子释放位置按 P 保存释放位。",
        "按 R 返回搜索位后进入下一篮；A/B 区域按项目约定切换起始姿态。",
        "完成后按 Q 保存，备份 ~/tl_mock_pick_place/basket_aruco.yaml。",
    ]:
        number(doc, text)
    code(doc, "cd ~/Desktop/TL\nbash scripts/teach_seed.sh")
    add_heading("192 个花篮编号", 2)
    add_table(doc, ["区域", "targetId", "内部编号 / ArUco ID"], [
        ("A1", "A1-01 ~ A1-24", "1~24 / 0~23"),
        ("A2", "A2-01 ~ A2-24", "25~48 / 24~47"),
        ("A3", "A3-01 ~ A3-24", "49~72 / 48~71"),
        ("A4", "A4-01 ~ A4-24", "73~96 / 72~95"),
        ("B1~B4", "B1-01 ~ B4-24", "97~192 / 96~191"),
    ], [1800, 3600, 3960])
    callout(doc, "码规格核对", "当前最新实体码应使用 DICT_4X4_250、ID 0~191、黑方块 58 mm。工程内部分旧脚本仍默认 54 mm/0.054 m，示教和识别前必须统一为 0.058 m，否则会检测失败或产生比例偏差。", WARN, "7A5A00")
    add_heading("MQTT 命令", 2)
    code(doc, "mosquitto_pub -h <broker> -t node/arm/01/down -q 1 \\\n  -m '{\"cmd\":\"seed\",\"targetId\":\"A1-03\"}'\n\n# 查看反馈\nmosquitto_sub -h <broker> -t node/arm/01/up -q 1 -v")
    add_table(doc, ["retCode", "含义"], [("0", "完成成功"), ("1", "收到命令并开始动作"), ("2", "忙/位置被占"), ("3", "识别、运动或放置失败，查看 message")], [1800, 7560])

    add_heading("程序边界")
    para(doc, "U 型叉子程序是独立工程，不调用灵巧手、不使用种子 MQTT 流程。它以花篮为对象，全程携篮执行接近、水平插入、竖直抬升、高位转运、临时位置停留和逆序放回。")
    add_heading("示教按键", 2)
    add_table(doc, ["按键", "记录内容", "要求"], [
        ("1~24/编号", "选择当前花篮", "先选编号再对准对应 ArUco"),
        ("B", "搜索位", "码稳定可见"),
        ("F", "接近位", "叉子在篮口外"),
        ("I", "水平插入位", "保持叉姿态、水平插入"),
        ("L", "竖直抬升位", "保持 XY 和姿态，只向上"),
        ("W/T", "转运点/临时持篮位", "花篮已在叉子上，先抬高再平移"),
        ("O/R/Q", "观测/返回/保存退出", "R 完成回位后再示教下一篮"),
    ], [1800, 3100, 4460])
    add_heading("执行流程", 2)
    for text in [
        "trim_pick：检索码位 → 叉子接近 → 水平插入 → 竖直抬升 → 高位转运 → 临时持篮位。",
        "携篮期间：花篮始终留在叉子上，active_transfer.yaml 保存原位和当前携篮状态。",
        "trim_place：从临时位抬升 → 回到原篮上方 → 下降 → 抽叉 → 回搜索位。",
    ]:
        number(doc, text)
    code(doc, "cd ~/Desktop/TL\nbash scripts/teach_fork.sh\n# 执行节点使用 TL_Handback 中的 fork_basket_cycle.py")
    callout(doc, "数据隔离", "种子程序使用 ~/tl_mock_pick_place/basket_aruco.yaml；叉子程序使用 ~/tl_fork_teach/fork_basket_teach.yaml 和 active_transfer.yaml。两个文件不能互换。", RISK, "9B1C1C")

    add_heading("当前状态")
    para(doc, "VLA（pi0、SmolVLA、GR00T 等）属于上层策略，不是当前 TL 工程的底层运动控制。当前工程已经具备驱动、相机、手眼变换、示教数据、灵巧手、U 型叉和 MQTT 接口，但尚未把 VLA 推理结果接入可验收的闭环。")
    add_heading("推荐部署路线", 2)
    add_table(doc, ["阶段", "内容"], [
        ("基线", "先用示教 + ArUco + 固定抓取参数跑通稳定流程，记录成功率和偏差。"),
        ("数据", "采集图像、关节、TCP、手部动作和任务结果，确保时间戳一致。"),
        ("训练", "在外部 GPU/训练服务器上微调，Jetson 主要负责推理和实时控制。"),
        ("上线", "VLA 只能输出经过工作空间、速度、姿态、力度和急停约束的动作目标。"),
    ], [2200, 7160])
    callout(doc, "结论", "现阶段不要用 VLA 替代已经验证的示教和坐标变换。先完成基线抓取、放置和安全边界，再把 VLA 接到“目标位姿/动作意图”接口上。", WARN, "7A5A00")

    add_heading("运动与设备安全")
    for text in [
        "物理 E-stop 始终在手边；程序 estop 只是软件锁存，不等于机械臂断电。",
        "运动前清空工作区，确认相机线缆、灵巧手、U 型叉子和花篮不会碰撞。",
        "不要同时启动两个运动节点；出现 D435 timeout、IK failed、CoordTransform failed 或抖动时立即停止。",
        "真实灵巧手必须先空载验证张开、力度 1 和力度 5，接触抓取时从低速度、低力度和较大间隙开始。",
        "叉子携篮时先确认插入深度和篮体稳定性；中断后根据现场状态处理 active_transfer.yaml，不可盲删。",
        "修改标定、示教、MQTT 和启动参数前先备份，保留最后一次可运行版本。",
    ]:
        bullet(doc, text)
    add_heading("现场启动检查", 2)
    add_table(doc, ["项目", "通过标准"], [
        ("驱动", "连接成功，/joint_states 和 /tcp_pose 持续更新"),
        ("相机", "D435 稳定出彩色/深度帧，只有一个程序占用"),
        ("标定", "正式文件 accepted: true，九点误差满足要求"),
        ("ArUco", "DICT_4X4_250、58 mm、ID 与 targetId 一一对应"),
        ("示教", "目标花篮有完整 search/approach/release 或 fork 关键点"),
        ("通信", "MQTT 已连接，测试只发送一条命令并能收到反馈"),
    ], [2600, 6760])
    add_heading("常见故障", 2)
    add_table(doc, ["现象", "处理"], [
        ("D435 frame timeout", "关闭其他相机进程，换 USB3 口并重启节点"),
        ("MQTT reason=5", "检查 broker 用户名和密码"),
        ("retCode=3", "查看反馈 message，确认示教文件、码规格和 IK"),
        ("taught marker 数量不对", "检查是否被旧版 basket_aruco.py 或示教文件覆盖"),
        ("机械臂抖动", "确认只有一个 ServoJ 发布者，先降低速度，不要降低发布频率"),
        ("叉篮启动不动", "确认 active_transfer.yaml 是否表示叉上仍有花篮"),
    ], [3000, 6360])
    add_heading("参考路径", 2)
    for item in [
        "C:\\Users\\DragonAfterAll\\Desktop\\TL\\README.md",
        "C:\\Users\\DragonAfterAll\\Desktop\\TL\\程序说明文档.md",
        "C:\\Users\\DragonAfterAll\\Desktop\\TL\\TL_Dobot\\docs\\handeye_precision_pipeline_zh.md",
        "C:\\Users\\DragonAfterAll\\Desktop\\TL\\TL_Dobot\\docs\\roh_hand_pick_place_workflow_zh.md",
        "C:\\Users\\DragonAfterAll\\Desktop\\TL\\TL_Handback\\docs\\fork_basket_teach_zh.md",
    ]:
        bullet(doc, item)
    doc.save(str(OUTPUT))
    print(str(OUTPUT))


if __name__ == "__main__":
    build()
