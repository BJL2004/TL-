#!/usr/bin/env python3
"""Markdown → DOCX 转换器（中文文档专用，排版可控）

特点：
- 表格：满宽(100%)、Table Grid 边框、表头加粗+浅蓝底、单元格 10pt
- 正文：宋体 11pt、行距 1.3、两端对齐、首行缩进 2 字符
- 标题：微软雅黑加深，章节自动分页；使用内置 Heading 样式（目录可识别）
- 代码块：Consolas 9pt、浅灰底、紧凑行距
- 封面块：标题+副标题+说明，居中；真目录域（打开时提示更新）+ 更新提示行
- 页脚：居中页码
- 行内：**加粗**、`行内代码`（浅灰底）

用法：
    python md2docx.py 输入.md 输出.docx ["文档主标题"]
"""
import re
import sys

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

EA_BODY = "宋体"
EA_HEAD = "微软雅黑"
ASCII_BODY = "Times New Roman"
ASCII_CODE = "Consolas"


# ---------------- 低层工具 ----------------
def set_run_fonts(run, ascii_font, ea_font, size=None, bold=None, color=None):
    run.font.name = ascii_font
    if size is not None:
        run.font.size = Pt(size)
    if bold is not None:
        run.font.bold = bold
    if color is not None:
        run.font.color.rgb = RGBColor(*color)
    rpr = run._element.get_or_add_rPr()
    rfonts = rpr.find(qn("w:rFonts"))
    if rfonts is None:
        rfonts = OxmlElement("w:rFonts")
        rpr.append(rfonts)
    rfonts.set(qn("w:ascii"), ascii_font)
    rfonts.set(qn("w:hAnsi"), ascii_font)
    rfonts.set(qn("w:eastAsia"), ea_font)


def shade(element_pr, fill):
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), fill)
    element_pr.append(shd)


def shade_paragraph(p, fill):
    shade(p._p.get_or_add_pPr(), fill)


def shade_run(run, fill):
    shade(run._element.get_or_add_rPr(), fill)


def add_page_field(paragraph):
    run = paragraph.add_run()
    fld_begin = OxmlElement("w:fldChar")
    fld_begin.set(qn("w:fldCharType"), "begin")
    instr = OxmlElement("w:instrText")
    instr.set(qn("xml:space"), "preserve")
    instr.text = " PAGE "
    fld_end = OxmlElement("w:fldChar")
    fld_end.set(qn("w:fldCharType"), "end")
    run._element.append(fld_begin)
    run._element.append(instr)
    run._element.append(fld_end)
    return run


def add_toc_field(paragraph):
    run = paragraph.add_run()
    fld_begin = OxmlElement("w:fldChar")
    fld_begin.set(qn("w:fldCharType"), "begin")
    instr = OxmlElement("w:instrText")
    instr.set(qn("xml:space"), "preserve")
    instr.text = ' TOC \\o "1-3" \\h \\z \\u '
    fld_sep = OxmlElement("w:fldChar")
    fld_sep.set(qn("w:fldCharType"), "separate")
    placeholder = OxmlElement("w:t")
    placeholder.text = "（目录占位：在 Word 中右键 → 更新域）"
    fld_end = OxmlElement("w:fldChar")
    fld_end.set(qn("w:fldCharType"), "end")
    for el in (fld_begin, instr, fld_sep, placeholder, fld_end):
        run._element.append(el)
    return run


def enable_update_fields(doc):
    settings = doc.settings.element
    update = OxmlElement("w:updateFields")
    update.set(qn("w:val"), "true")
    settings.append(update)


# ---------------- 行内格式 ----------------
INLINE_RE = re.compile(r"(\*\*.+?\*\*|`[^`]+`)")


def add_inline(paragraph, text, base_size=11, ea=EA_BODY, ascii_font=ASCII_BODY):
    for token in INLINE_RE.split(text):
        if not token:
            continue
        if token.startswith("**") and token.endswith("**") and len(token) > 4:
            run = paragraph.add_run(token[2:-2])
            set_run_fonts(run, ascii_font, ea, base_size, bold=True)
        elif token.startswith("`") and token.endswith("`") and len(token) > 2:
            run = paragraph.add_run(token[1:-1])
            set_run_fonts(run, ASCII_CODE, EA_BODY, base_size - 1)
            shade_run(run, "F2F2F2")
        else:
            run = paragraph.add_run(token)
            set_run_fonts(run, ascii_font, ea, base_size)


# ---------------- Markdown 解析 ----------------
def parse_markdown(lines):
    blocks = []
    i = 0
    n = len(lines)
    while i < n:
        line = lines[i].rstrip()
        if not line.strip():
            i += 1
            continue
        # 代码块
        if line.strip().startswith("```"):
            i += 1
            code_lines = []
            while i < n and not lines[i].strip().startswith("```"):
                code_lines.append(lines[i].rstrip("\n"))
                i += 1
            i += 1
            blocks.append(("code", code_lines))
            continue
        # 分隔线
        if re.fullmatch(r"-{3,}\s*", line):
            i += 1
            continue
        # 标题
        m = re.match(r"^(#{1,4})\s+(.*)$", line)
        if m:
            level = len(m.group(1))
            blocks.append((f"h{level}", m.group(2).strip()))
            i += 1
            continue
        # 表格
        if line.lstrip().startswith("|"):
            table_lines = []
            while i < n and lines[i].lstrip().startswith("|"):
                table_lines.append(lines[i].strip())
                i += 1
            blocks.append(("table", table_lines))
            continue
        # 引用
        if line.lstrip().startswith(">"):
            quote_lines = []
            while i < n and lines[i].lstrip().startswith(">"):
                quote_lines.append(lines[i].lstrip()[1:].strip())
                i += 1
            blocks.append(("quote", quote_lines))
            continue
        # 列表
        m = re.match(r"^(\s*)([-*]|\d+\.)\s+(.*)$", line)
        if m:
            indent = len(m.group(1))
            marker = m.group(2)
            blocks.append(("bullet" if marker in ("-", "*") else "number",
                           (indent, m.group(3).strip())))
            i += 1
            continue
        # 普通段落（连续行合并）
        para = [line.strip()]
        i += 1
        while i < n and lines[i].strip() and not re.match(
            r"^\s*(#{1,4}\s|```|\||>|\s*([-*]|\d+\.)\s)", lines[i]
        ) and not re.fullmatch(r"-{3,}\s*", lines[i].strip()):
            para.append(lines[i].strip())
            i += 1
        blocks.append(("para", " ".join(para)))
    return blocks


def split_table_row(row_text):
    cells = row_text.strip().strip("|").split("|")
    return [c.strip() for c in cells]


def is_separator_row(cells):
    return all(re.fullmatch(r":?-{2,}:?", c) for c in cells if c != "")


# ---------------- 文档构建 ----------------
def build_doc(md_path, out_path, main_title=None):
    with open(md_path, encoding="utf-8") as f:
        lines = f.read().splitlines()
    blocks = parse_markdown(lines)

    doc = Document()

    # 页面：A4，边距
    sec = doc.sections[0]
    sec.page_width, sec.page_height = Cm(21.0), Cm(29.7)
    sec.top_margin = sec.bottom_margin = Cm(2.4)
    sec.left_margin = sec.right_margin = Cm(2.6)

    # 样式
    normal = doc.styles["Normal"]
    normal.font.name = ASCII_BODY
    normal.font.size = Pt(11)
    normal.element.rPr.rFonts.set(qn("w:eastAsia"), EA_BODY)
    normal.paragraph_format.line_spacing = 1.3
    normal.paragraph_format.space_after = Pt(4)
    normal.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY

    head_specs = {1: (17, True), 2: (14.5, True), 3: (12.5, True), 4: (12, True)}
    for level, (size, bold) in head_specs.items():
        st = doc.styles[f"Heading {level}"]
        st.font.name = EA_HEAD
        st.font.size = Pt(size)
        st.font.bold = bold
        st.font.color.rgb = RGBColor(0x1F, 0x1F, 0x1F)
        rpr = st.element.get_or_add_rPr()
        rfonts = rpr.find(qn("w:rFonts"))
        if rfonts is None:
            rfonts = OxmlElement("w:rFonts")
            rpr.append(rfonts)
        for attr in ("w:ascii", "w:hAnsi", "w:eastAsia"):
            rfonts.set(qn(attr), EA_HEAD)
        st.paragraph_format.line_spacing = 1.3
        st.paragraph_format.space_before = Pt(14 if level == 1 else 10)
        st.paragraph_format.space_after = Pt(8)

    # 页脚页码
    footer_p = sec.footer.paragraphs[0]
    footer_p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    frun = add_page_field(footer_p)
    set_run_fonts(frun, ASCII_BODY, EA_BODY, 9, color=(0x88, 0x88, 0x88))

    # 封面块
    title_text = main_title
    body_blocks = blocks
    if blocks and blocks[0][0] == "h1":
        title_text = title_text or blocks[0][1]
        body_blocks = blocks[1:]
    if title_text:
        p = doc.add_paragraph()
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        p.paragraph_format.space_before = Pt(90)
        p.paragraph_format.space_after = Pt(10)
        run = p.add_run(title_text)
        set_run_fonts(run, EA_HEAD, EA_HEAD, 22, bold=True, color=(0x1F, 0x1F, 0x1F))
    # 封面后的引用（版本/说明）居中小字
    while body_blocks and body_blocks[0][0] == "quote":
        for qline in body_blocks[0][1]:
            p = doc.add_paragraph()
            p.alignment = WD_ALIGN_PARAGRAPH.CENTER
            p.paragraph_format.space_after = Pt(2)
            run = p.add_run(qline)
            set_run_fonts(run, ASCII_BODY, EA_BODY, 10, color=(0x66, 0x66, 0x66))
        body_blocks = body_blocks[1:]

    # 目录
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_before = Pt(24)
    p.paragraph_format.space_after = Pt(10)
    run = p.add_run("目 录")
    set_run_fonts(run, EA_HEAD, EA_HEAD, 16, bold=True, color=(0x1F, 0x1F, 0x1F))
    toc_p = doc.add_paragraph()
    add_toc_field(toc_p)
    hint = doc.add_paragraph()
    hint.paragraph_format.space_before = Pt(6)
    hrun = hint.add_run("（首次打开：右键目录 → 更新域 → 更新整个目录）")
    set_run_fonts(hrun, ASCII_BODY, EA_BODY, 9, color=(0x99, 0x99, 0x99))

    # 正文

    for block in body_blocks:
        kind = block[0]
        if kind in ("h1", "h2", "h3", "h4"):
            # '# ' 是文档标题（已用于封面），正文标题整体上移一级：
            # md 的 ## → Heading 1（章），### → Heading 2（节），#### → Heading 3
            level = max(1, int(kind[1]) - 1)
            if level == 1:
                # 章标题自带分页（避免空分页段）
                h = doc.add_heading(block[1], level=1)
                h.paragraph_format.page_break_before = True
            else:
                doc.add_heading(block[1], level=level)
        elif kind == "para":
            p = doc.add_paragraph()
            p.paragraph_format.first_line_indent = Pt(22)
            add_inline(p, block[1])
        elif kind == "quote":
            for qline in block[1]:
                p = doc.add_paragraph()
                p.paragraph_format.left_indent = Cm(0.5)
                p.paragraph_format.space_after = Pt(2)
                add_inline(p, qline, base_size=10)
                for run in p.runs:
                    run.font.color.rgb = RGBColor(0x59, 0x59, 0x59)
        elif kind == "bullet":
            indent, text = block[1]
            p = doc.add_paragraph()
            lvl = 1 if indent >= 2 else 0
            p.paragraph_format.left_indent = Cm(0.75 + 0.6 * lvl)
            p.paragraph_format.first_line_indent = Cm(-0.45)
            add_inline(p, ("• " if lvl == 0 else "– ") + text)
        elif kind == "number":
            indent, text = block[1]
            p = doc.add_paragraph()
            p.paragraph_format.left_indent = Cm(0.75)
            p.paragraph_format.first_line_indent = Cm(-0.45)
            add_inline(p, "▪ " + text)
        elif kind == "code":
            code_lines = block[1]
            for idx, cline in enumerate(code_lines):
                p = doc.add_paragraph()
                p.paragraph_format.line_spacing = 1.05
                p.paragraph_format.left_indent = Cm(0.3)
                p.paragraph_format.space_before = Pt(4 if idx == 0 else 0)
                p.paragraph_format.space_after = Pt(4 if idx == len(code_lines) - 1 else 0)
                p.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.LEFT
                shade_paragraph(p, "F2F2F2")
                run = p.add_run(cline if cline else " ")
                set_run_fonts(run, ASCII_CODE, EA_BODY, 9)
        elif kind == "table":
            rows = [split_table_row(r) for r in block[1]]
            rows = [r for r in rows if not is_separator_row(r)]
            if not rows:
                continue
            cols = max(len(r) for r in rows)
            table = doc.add_table(rows=len(rows), cols=cols)
            table.style = "Table Grid"
            table.autofit = True
            # 满宽
            tblPr = table._tbl.tblPr
            tblW = tblPr.find(qn("w:tblW"))
            if tblW is None:
                tblW = OxmlElement("w:tblW")
                tblPr.append(tblW)
            tblW.set(qn("w:type"), "pct")
            tblW.set(qn("w:w"), "5000")
            for ri, row in enumerate(rows):
                for ci in range(cols):
                    cell = table.cell(ri, ci)
                    text = row[ci] if ci < len(row) else ""
                    cp = cell.paragraphs[0]
                    cp.paragraph_format.line_spacing = 1.15
                    cp.paragraph_format.space_before = Pt(2)
                    cp.paragraph_format.space_after = Pt(2)
                    cp.paragraph_format.alignment = WD_ALIGN_PARAGRAPH.LEFT
                    add_inline(cp, text, base_size=10)
                    if ri == 0:
                        for run in cp.runs:
                            run.font.bold = True
                        shade(cell._tc.get_or_add_tcPr(), "DDEBF7")
            doc.add_paragraph().paragraph_format.space_after = Pt(2)

    enable_update_fields(doc)
    doc.save(out_path)
    print(f"已生成: {out_path}  （块数: {len(body_blocks)}）")


if __name__ == "__main__":
    src = sys.argv[1]
    dst = sys.argv[2]
    title = sys.argv[3] if len(sys.argv) > 3 else None
    build_doc(src, dst, title)
