from pathlib import Path

from docx import Document
from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT, WD_TABLE_ALIGNMENT
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches, Pt, RGBColor


ROOT = Path(r"C:\Users\DragonAfterAll\Desktop\TL")
PATH = next(path for path in ROOT.glob("*.docx") if path.stat().st_size > 160000)
OUTPUT = ROOT / "TL_program_manual_vla_act.docx"
BLUE = "2E74B5"
DARK = "1F4D78"
TEXT = "1F2937"
FILL = "E8EEF5"
WARN = "FFF4D6"
RISK = "FCE8E6"


def set_font(run, size=10.5, color=TEXT, bold=None):
    run.font.name = "Calibri"
    run._element.get_or_add_rPr().rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
    run.font.size = Pt(size)
    run.font.color.rgb = RGBColor.from_string(color)
    if bold is not None:
        run.bold = bold


def set_margins(cell):
    tc_pr = cell._tc.get_or_add_tcPr()
    tc_mar = tc_pr.find(qn("w:tcMar"))
    if tc_mar is None:
        tc_mar = OxmlElement("w:tcMar")
        tc_pr.append(tc_mar)
    for side, value in (("top", 80), ("start", 120), ("bottom", 80), ("end", 120)):
        node = tc_mar.find(qn(f"w:{side}"))
        if node is None:
            node = OxmlElement(f"w:{side}")
            tc_mar.append(node)
        node.set(qn("w:w"), str(value))
        node.set(qn("w:type"), "dxa")


def set_geometry(table, widths):
    table.alignment = WD_TABLE_ALIGNMENT.LEFT
    table.autofit = False
    pr = table._tbl.tblPr
    layout = pr.find(qn("w:tblLayout"))
    if layout is None:
        layout = OxmlElement("w:tblLayout")
        pr.append(layout)
    layout.set(qn("w:type"), "fixed")
    tbl_w = pr.find(qn("w:tblW"))
    if tbl_w is None:
        tbl_w = OxmlElement("w:tblW")
        pr.append(tbl_w)
    tbl_w.set(qn("w:w"), str(sum(widths)))
    tbl_w.set(qn("w:type"), "dxa")
    ind = pr.find(qn("w:tblInd"))
    if ind is None:
        ind = OxmlElement("w:tblInd")
        pr.append(ind)
    ind.set(qn("w:w"), "120")
    ind.set(qn("w:type"), "dxa")
    grid = table._tbl.tblGrid
    for child in list(grid):
        grid.remove(child)
    for width in widths:
        col = OxmlElement("w:gridCol")
        col.set(qn("w:w"), str(width))
        grid.append(col)
    for row in table.rows:
        for idx, cell in enumerate(row.cells):
            cell.width = Inches(widths[idx] / 1440)
            cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
            set_margins(cell)


def put_cell(cell, value, bold=False, color=TEXT, size=9.2):
    cell.text = ""
    p = cell.paragraphs[0]
    p.paragraph_format.space_after = Pt(0)
    p.paragraph_format.line_spacing = 1.1
    r = p.add_run(str(value))
    set_font(r, size=size, color=color, bold=bold)


def add_table(doc, headers, rows, widths):
    table = doc.add_table(rows=1, cols=len(headers))
    set_geometry(table, widths)
    header = table.rows[0]
    tr_pr = header._tr.get_or_add_trPr()
    repeat = OxmlElement("w:tblHeader")
    repeat.set(qn("w:val"), "true")
    tr_pr.append(repeat)
    for idx, value in enumerate(headers):
        put_cell(header.cells[idx], value, True, DARK)
        shd = OxmlElement("w:shd")
        shd.set(qn("w:fill"), FILL)
        header.cells[idx]._tc.get_or_add_tcPr().append(shd)
    for row in rows:
        cells = table.add_row().cells
        for idx, value in enumerate(row):
            put_cell(cells[idx], value)
    return table


def add_callout(doc, label, text, fill=FILL, color=DARK):
    table = doc.add_table(rows=1, cols=1)
    set_geometry(table, [9360])
    cell = table.cell(0, 0)
    shd = OxmlElement("w:shd")
    shd.set(qn("w:fill"), fill)
    cell._tc.get_or_add_tcPr().append(shd)
    p = cell.paragraphs[0]
    p.paragraph_format.space_after = Pt(0)
    r = p.add_run(label + "：")
    set_font(r, bold=True, color=color)
    r = p.add_run(text)
    set_font(r)
    return table


def add_para(doc, text, style="Normal"):
    p = doc.add_paragraph(style=style)
    p.add_run(text)
    return p


def add_bullet(doc, text):
    return add_para(doc, text, "List Bullet")


def add_number(doc, text):
    return add_para(doc, text, "List Number")


def add_code(doc, text):
    return add_para(doc, text, "Code Block")


def remove_vla_body(doc, vla, safety):
    node = vla._p.getnext()
    while node is not None and node is not safety._p:
        nxt = node.getnext()
        node.getparent().remove(node)
        node = nxt


def move_before_safety(safety, nodes):
    for node in nodes:
        safety._p.addprevious(node)


def build():
    doc = Document(str(PATH))
    vla = next(p for p in doc.paragraphs if p.text == "VLA部署")
    safety = next(p for p in doc.paragraphs if p.text == "安全注意事项")
    remove_vla_body(doc, vla, safety)
    nodes = []

    def h(text):
        p = doc.add_heading(text, level=2)
        nodes.append(p._p)

    def p(text):
        item = add_para(doc, text)
        nodes.append(item._p)

    def b(text):
        item = add_bullet(doc, text)
        nodes.append(item._p)

    def n(text):
        item = add_number(doc, text)
        nodes.append(item._p)

    def c(text):
        item = add_code(doc, text)
        nodes.append(item._p)

    def t(headers, rows, widths):
        item = add_table(doc, headers, rows, widths)
        nodes.append(item._tbl)

    def note(label, text, fill=FILL, color=DARK):
        item = add_callout(doc, label, text, fill, color)
        nodes.append(item._tbl)

    h("1. VLA、VA 与 ACT 的概念区别")
    p("VLA 是 Vision-Language-Action 的缩写，输入通常包括图像、机器人状态和自然语言任务指令，输出机器人动作序列。它解决的是“看懂任务并采取动作”的问题，适合目标、场景和任务描述变化较大的应用。")
    p("VA 可以理解为 Vision-Action policy（视觉-动作策略）：输入图像、深度、关节/TCP 等观测，直接输出动作，不额外引入语言理解。对于本项目这种任务固定、花篮编号由 MQTT 给出、抓取姿态和放置流程已有示教的系统，VA 比完整 VLA 更容易先落地。")
    p("ACT 是 Action Chunking with Transformers。它属于模仿学习策略，不是通用语言模型。ACT 根据最近若干帧图像和机器人状态，一次预测一段未来动作 chunk，再采用滚动时域执行。它适合“一个任务、多次示教、动作连续”的场景，例如固定高度的种子抓取与放置。")
    t(["类型", "输入", "输出", "对当前 TL 的适配"], [
        ("VLA", "图像/状态/语言", "动作块或末端/关节动作", "适合多任务和自然语言，但算力、数据和部署复杂度最高"),
        ("VA", "图像/深度/状态/任务 ID", "末端位姿、关节增量或离散阶段", "适合当前固定流程，建议作为第一版学习策略"),
        ("ACT", "图像/状态/可选任务 ID", "未来多步动作 chunk", "适合单任务或少量任务，优先用于基线"),
        ("传统示教", "ArUco、深度、示教文件", "确定性轨迹", "当前最稳定的底层基线，不应被模型绕过"),
    ], [1300, 2400, 2600, 3060])
    note("选择建议", "当前项目不要一开始就把所有任务交给完整 VLA。推荐顺序是：传统示教基线 → ACT/VA 单任务策略 → 受限多任务 VA → 最后再接 pi0、SmolVLA 或 GR00T。", WARN, "7A5A00")

    h("2. pi0、SmolVLA、GR00T 与 ACT 的定位")
    p("pi0、SmolVLA 和 GR00T 都可以作为 VLA/多模态策略方向，但它们的输入格式、机器人 embodiment、动作空间和部署工具不同，不能直接把一个模型的 checkpoint 放进另一个模型的推理脚本。ACT/VA 则更像是本项目自己的任务策略，模型输出应适配 TCB605 和 ROH 灵巧手的动作接口。")
    t(["模型/策略", "主要特点", "适合本项目的用途", "主要风险"], [
        ("pi0 / openpi", "视觉、语言和动作联合策略，通常通过微调适配机器人", "将已验证的种子/放置示教扩展成语言条件任务", "模型和训练依赖重；必须建立 TCB605 的动作/观测适配层"),
        ("SmolVLA", "轻量化 VLA 思路，强调较低资源下的视觉语言动作推理", "在外部 GPU 训练后尝试边缘推理或远程推理", "轻量不等于无需适配；相机尺寸、动作维度和归一化必须一致"),
        ("GR00T", "面向通用机器人技能的基础 VLA，可通过新 embodiment 数据微调", "作为后续多任务策略和远程 policy server 方案", "模型较重；需要新机器人 embodiment、数据格式和推理服务"),
        ("ACT", "Transformer 模仿学习，一次预测未来动作块", "固定俯视抓取、平抓、固定花篮放置的第一版学习基线", "泛化到新物体/新布局弱，依赖示教分布"),
        ("VA/BC", "视觉状态到动作的行为克隆，不输入自然语言", "固定任务、多类别种球和有限花篮位置", "多任务扩展需要 task ID 或阶段标签"),
    ], [1600, 2500, 3100, 2160])
    p("工程选择上，ACT/VA 的优点是输出空间可以严格限定为 TCB605 的关节增量或掌心相对位姿；VLA 的优点是能处理语言条件、多任务和更大的视觉变化。无论使用哪一种，模型输出都必须经过本地动作约束，不允许直接写入电机或数字输出。")
    p("官方资料建议在真实机器人部署中先做 open-loop 验证，再做 closed-loop 验证；GR00T 也支持把模型推理放在独立服务器、机器人端只保留轻量 client 的 server-client 结构。这正适合当前 Jetson 作为控制端、外部 GPU 作为模型端的分工。")

    h("3. 面向当前工程的总体部署架构")
    c("D435 / joint_states / tcp_pose / hand_state\n                 │\n                 ▼\n        observation_adapter\n  图像同步、裁剪、归一化、状态拼接\n                 │\n                 ▼\n     ACT / VA / VLA policy client\n       本地推理或远程 policy server\n                 │  action chunk\n                 ▼\n        action_guard（必须保留）\n  工作空间、关节、速度、姿态、阶段、急停\n                 │\n       ┌─────────┴─────────┐\n       ▼                   ▼\n  ServoJ executor       hand controller\n  TCB605 100 Hz        张开/力度1/力度5")
    t(["组件", "放置位置", "职责"], [
        ("数据采集/适配", "Jetson", "同步 RGB/深度/关节/TCP/手部状态，统一时间戳和坐标系"),
        ("模型推理", "外部 GPU 或 Jetson", "ACT/VA 可优先本地；大型 VLA 建议远程 policy server"),
        ("动作安全层", "Jetson 本地", "拒绝越界、速度异常、姿态突变和急停状态动作"),
        ("运动执行", "Jetson + 天链驱动", "把安全动作插值为 100 Hz ServoJ 或厂商可验证接口"),
        ("灵巧手控制", "Jetson 本地", "模型只给抓取意图，手部节点决定张开、力度 1/力度 5 和保护动作"),
    ], [2200, 1900, 5260])
    note("边界", "模型服务可以失联，但机械臂控制端必须能超时停止；任何模型输出都不能绕过本地 action_guard、物理 E-stop、手部保护和 MQTT 状态机。", RISK, "9B1C1C")

    h("4. 数据采集：从示教数据变成学习数据")
    p("现有示教文件主要保存姿态和花篮相对位姿，能够驱动确定性流程，但还不足以直接训练 ACT/VA/VLA。学习数据必须把每个时刻的观测、动作、任务阶段和最终结果绑定起来。")
    t(["字段", "建议内容", "用途"], [
        ("image", "D435 彩色图像；深度可选但要和 RGB 对齐", "视觉输入，记录分辨率、曝光和相机序列号"),
        ("state", "joint position/velocity、TCP 位姿、手部位置/状态", "让策略知道当前机械臂和灵巧手状态"),
        ("action", "关节增量、末端相对位姿或动作 chunk", "必须固定维度、单位和坐标系"),
        ("task", "seed_pick、seed_place、fork_pick 等阶段", "区分任务和阶段，避免模型学混"),
        ("target", "targetId、ArUco ID、当前花篮位姿", "将视觉目标与任务目标绑定"),
        ("result", "success、failure_reason、碰撞/压碰标记", "过滤坏样本和做评估"),
    ], [1700, 4700, 2960])
    add_number(doc, "每条 episode 从明确的起始观测开始，完整包含接近、抓取、抬升、转运、放置和结束状态；不要只截取成功动作的中间几帧。")
    nodes.append(doc.paragraphs[-1]._p)
    add_number(doc, "记录 20~50 Hz 的观测和动作，所有 RGB、深度、关节、TCP、灵巧手状态使用同一时间戳或可追溯的时间同步关系。")
    nodes.append(doc.paragraphs[-1]._p)
    add_number(doc, "训练集、验证集、测试集按 episode 或采集日期拆分，不要把同一段轨迹的相邻帧随机分到不同集合。")
    nodes.append(doc.paragraphs[-1]._p)
    add_number(doc, "对失败样本标明失败原因：目标偏移、手部压碰、抓空、掉落、ArUco 丢失、IK 拒绝或通信超时。")
    nodes.append(doc.paragraphs[-1]._p)
    note("数据规模建议", "当前固定任务可先采集每个动作 30~50 个完整 episode 做 ACT/VA 基线；如果训练 GR00T 这类通用策略，官方部署资料给出的单任务起点通常至少 100 个有效 episode，较稳定的训练往往需要 200 个以上。实际数量仍取决于任务变化和数据质量。", WARN, "7A5A00")

    h("5. ACT 部署方案：建议先完成的第一条学习路线")
    p("ACT 不需要先解决自然语言理解，最适合把当前已经跑通的固定俯视抓取流程学习成一个闭环策略。第一版建议只做一个任务：固定观测位、固定相机安装、固定抓取姿态、固定高度范围、假手或低风险工具。")
    add_heading = lambda text: (nodes.append(doc.add_heading(text, level=3)._p))
    add_heading("5.1 ACT 的动作空间")
    for text in [
        "推荐输出关节相对增量或末端相对位姿，不推荐第一版直接输出电机/DO 指令。",
        "种子任务可将动作拆为 arm_action 和 hand_intent：arm_action 为 TCB605 关节/末端目标，hand_intent 只包含 open、hold1、grasp5、release 等离散意图。",
        "由本地 hand controller 把 hand_intent 转换为实际手部位置，模型不能直接改变保护上限。",
        "使用 action chunk 预测未来 8~32 个动作点，再用滚动时域每次执行前若干点并重新推理，避免一次预测错误持续太久。",
    ]:
        b(text)
    add_heading("5.2 ACT 训练与验证顺序")
    for text in [
        "离线回放：只把测试图像送入模型，检查预测轨迹是否接近示教轨迹，不发送机械臂指令。",
        "无手/假手：允许机械臂在高间隙运行，只验证目标 XY、姿态和轨迹平滑性。",
        "低速真实手：限制 ServoJ 速度、下探深度和手部力度，先验证抓空和压碰保护。",
        "闭环任务：在固定目标、轻微位置变化和不同物体外观下分别统计成功率，确认不是只记住单张图。",
    ]:
        n(text)
    c("# 伪代码：模型只给意图，底层仍由 TL 控制\nobs = adapter.read_rgb_depth_state()\naction_chunk = act_policy.predict(obs)\nfor action in action_chunk:\n    checked = action_guard.check(action, current_state)\n    if not checked.ok:\n        executor.stop(); hand_controller.hold_level1(); break\n    executor.servoj(checked.action, rate_hz=100)")

    h("6. VA 部署方案：适合当前固定任务的轻量策略")
    p("VA 不输入自然语言，而是用任务 ID、目标编号或当前流程阶段作为条件。例如把 seed_pick、seed_place、trim_pick、trim_place 分成不同 policy head，或者把 phase_id 拼到状态向量中。这样可以减少大语言模型开销，也能让动作输出更容易受控。")
    t(["VA 方案", "输入", "适用场景"], [
        ("单任务 BC", "RGB + joint/TCP", "固定俯视抓取或固定放置，最容易验证"),
        ("多阶段 VA", "RGB + state + phase_id", "接近/抓取/抬升/放置使用同一策略"),
        ("目标条件 VA", "RGB + state + target pose/ID", "目标位置有变化，但坐标变换仍由传统视觉提供"),
        ("双策略架构", "视觉策略给目标，传统轨迹执行", "最适合当前工程，保留 ArUco 和动作保护"),
    ], [2400, 3300, 3660])
    p("推荐使用“双策略架构”：ArUco/深度模块负责可靠的目标定位，VA 只负责在目标附近选择接近方向、抓取时机或动作微调。这样模型不需要重新学习相机到基座的全部几何关系，错误也更容易被 action_guard 拦截。")

    h("7. VLA 部署方案：pi0、SmolVLA、GR00T")
    p("完整 VLA 适用于“同一系统中任务、目标、语言指令和场景变化较大”的情况。例如让操作者用语言指定“把 A1-03 的种球放入指定花篮”，模型同时理解视觉目标、任务意图和动作策略。但对于当前项目，VLA 必须先被限制在已定义的任务集合中。")
    add_heading("7.1 适配 TCB605 的 embodiment 接口")
    for text in [
        "定义 observation schema：rgb、可选 depth、joint position、TCP pose、ROH 手状态、当前 task/phase 和 targetId。",
        "定义 action schema：推荐 6 维关节目标或 6D 末端相对位姿 + hand_intent；明确 mm/rad、坐标系、时间间隔和 action horizon。",
        "建立 TCB605 的 action decoder：把模型输出转换成 ServoJ 轨迹或受限 MoveJ，不直接将模型张量送给厂商驱动。",
        "建立 data processor：统一相机尺寸、颜色通道、归一化、状态字段名、关节顺序和缺失值处理。训练和部署必须使用同一 processor 配置。",
    ]:
        n(text)
    add_heading("7.2 本地推理与远程推理")
    t(["方式", "Jetson 端", "适合情况", "注意事项"], [
        ("本地推理", "相机、模型、控制全部在板上", "ACT/VA、量化后的轻量 VLA", "实时性好，但要实际测显存、延迟和温度"),
        ("远程 policy server", "Jetson 采集/执行，外部 GPU 推理", "pi0/较大 SmolVLA/GR00T", "网络超时必须触发保持或停止，不能阻塞急停"),
        ("混合模式", "本地传统定位 + 远程高层策略", "让 VLA 选策略，ArUco/底层负责几何", "需要明确谁拥有最终动作权限"),
    ], [1900, 2200, 2600, 2660])
    p("GR00T 官方部署文档支持 direct policy 和 server-client 两种模式，并建议将计算密集型推理与机器人端控制解耦。对于本项目的 Jetson Orin NX 16 GB，不能只看能否加载模型，还必须实测冷启动时间、单次推理延迟、连续运行温度、网络抖动和 action chunk 是否能覆盖控制频率。")
    add_heading("7.3 VLA 上线流程")
    for text in [
        "离线：使用录制数据做 open-loop 预测，确认模型输出维度、单位、关节顺序和动作方向。",
        "影子模式：真实采集相机和关节，但模型只输出日志，不移动机械臂；记录每次预测与传统目标的差异。",
        "高位空跑：使用假手/无手，限制工作区和速度，模型输出只允许在已验证的观测位附近执行。",
        "真实灵巧手：先使用力度 1 保护和低速，模型只控制接近和阶段切换，抓取力度仍由手部保护节点管理。",
        "MQTT 闭环：MQTT 只负责任务命令和结果反馈，不能让远程消息绕过本地 action_guard 或急停状态。",
    ]:
        n(text)
    c("# 远程推理超时策略\naction = policy_client.predict(observation, timeout_sec=0.20)\nif action.timeout:\n    executor.stop_or_hold()\n    hand_controller.hold_level1()\n    publish_feedback(ret_code=3, message=\"policy timeout\")")

    h("8. 当前项目的推荐实施顺序")
    t(["阶段", "目标", "完成标准"], [
        ("0. 确定性基线", "示教 + ArUco + ServoJ/灵巧手", "固定任务连续运行，记录成功率和 XY/Z 偏差"),
        ("1. ACT", "学习固定俯视抓取/放置", "离线预测正确，假手高位运行无碰撞"),
        ("2. VA", "加入物体外观、高度和少量任务阶段变化", "不同对象和目标位置下保持可接受成功率"),
        ("3. 轻量 VLA", "加入语言或 targetId 条件", "语言/编号只改变任务选择，不破坏动作约束"),
        ("4. 大型 VLA", "pi0/GR00T 远程或量化部署", "网络、模型、控制和急停全链路通过闭环验收"),
    ], [2100, 3800, 3460])
    note("最终结论", "你的当前工程最适合先做 ACT 或 VA：它们能复用已有示教和固定流程，也更适合 Jetson 端实时控制。VLA 的价值在于多任务和语言泛化，建议在数据、动作接口和安全层稳定后再接入；不是把现有程序直接替换成一个模型。", FILL, DARK)

    h("9. 官方参考资料")
    for text in [
        "Physical Intelligence openpi：https://github.com/Physical-Intelligence/openpi",
        "NVIDIA Isaac GR00T：https://github.com/NVIDIA/Isaac-GR00T",
        "GR00T real-world deployment：https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/real_world_deployment.md",
        "GR00T new embodiment fine-tuning：https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/finetune_new_embodiment.md",
        "ACT 论文：Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware（arXiv:2304.13705）",
    ]:
        b(text)

    move_before_safety(safety, nodes)
    doc.save(str(OUTPUT))
    print(OUTPUT)


if __name__ == "__main__":
    build()
