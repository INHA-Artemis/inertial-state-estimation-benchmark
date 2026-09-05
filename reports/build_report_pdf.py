#!/usr/bin/env python3
"""Build the Korean report as an A4 PDF."""

from __future__ import annotations

import base64
import html
import io
import re
from pathlib import Path

import mistune
from bs4 import BeautifulSoup, NavigableString, Tag
from PIL import Image as PillowImage
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.platypus import (
    HRFlowable,
    Image,
    ListFlowable,
    ListItem,
    LongTable,
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    TableStyle,
)


ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "KRISO_STATE_ESTIMATION_REPORT.md"
TARGET = ROOT / "KRISO_STATE_ESTIMATION_REPORT.pdf"
def register_fonts() -> None:
    # ReportLab's standard Korean CID font renders reliably with Poppler and
    # avoids the unsupported PostScript outlines in Ubuntu's Noto CJK TTC.
    pdfmetrics.registerFont(UnicodeCIDFont("HYSMyeongJo-Medium"))
    pdfmetrics.registerFontFamily(
        "HYSMyeongJo-Medium",
        normal="HYSMyeongJo-Medium",
        bold="HYSMyeongJo-Medium",
        italic="HYSMyeongJo-Medium",
        boldItalic="HYSMyeongJo-Medium",
    )


def inline_markup(node: Tag | NavigableString) -> str:
    if isinstance(node, NavigableString):
        # HYSMyeongJo lacks U+00B7 even though it covers Korean text.  A slash
        # preserves the intended grouping without producing a missing glyph.
        return html.escape(str(node).replace("·", "/"))
    children = "".join(inline_markup(child) for child in node.children)
    if node.name in {"strong", "b"}:
        return f"<b>{children}</b>"
    if node.name in {"em", "i"}:
        return f"<i>{children}</i>"
    if node.name == "code":
        return f'<font name="HYSMyeongJo-Medium" size="7.2">{children}</font>'
    if node.name == "a":
        href = html.escape(node.get("href", ""), quote=True)
        return f'<link href="{href}" color="#0c658c">{children}</link>'
    if node.name == "br":
        return "<br/>"
    return children


def paragraph_from_tag(tag: Tag, style: ParagraphStyle) -> Paragraph:
    return Paragraph("".join(inline_markup(child) for child in tag.children), style)


def image_flowable(tag: Tag) -> Image:
    source = tag.get("src", "")
    if source.startswith("data:image/"):
        payload = base64.b64decode(source.split(",", 1)[1])
        stream = io.BytesIO(payload)
    else:
        stream = io.BytesIO((ROOT / source).read_bytes())
    with PillowImage.open(stream) as raster:
        width_px, height_px = raster.size
    stream.seek(0)
    max_width, max_height = 160 * mm, 178 * mm
    scale = min(max_width / width_px, max_height / height_px)
    return Image(stream, width=width_px * scale, height=height_px * scale)


def table_flowable(tag: Tag, body_style: ParagraphStyle):
    raw_rows = []
    header_rows = 0
    for row in tag.find_all("tr"):
        cells = row.find_all(["th", "td"], recursive=False)
        if not cells:
            continue
        if any(cell.name == "th" for cell in cells):
            header_rows += 1
        raw_rows.append(cells)
    column_count = max(len(row) for row in raw_rows)
    font_size = 6.2 if column_count >= 7 else 7.0 if column_count >= 5 else 7.7
    cell_style = ParagraphStyle(
        "TableCell",
        parent=body_style,
        fontSize=font_size,
        leading=font_size * 1.35,
        spaceAfter=0,
    )
    data = []
    for cells in raw_rows:
        rendered = [paragraph_from_tag(cell, cell_style) for cell in cells]
        rendered.extend([""] * (column_count - len(rendered)))
        data.append(rendered)
    table = LongTable(
        data,
        colWidths=[(160 * mm) / column_count] * column_count,
        repeatRows=header_rows,
        hAlign="CENTER",
    )
    table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, header_rows - 1), colors.HexColor("#1d5b75")),
                ("TEXTCOLOR", (0, 0), (-1, header_rows - 1), colors.white),
                ("FONTNAME", (0, 0), (-1, header_rows - 1), "HYSMyeongJo-Medium"),
                ("GRID", (0, 0), (-1, -1), 0.35, colors.HexColor("#9baab3")),
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
                ("LEFTPADDING", (0, 0), (-1, -1), 3),
                ("RIGHTPADDING", (0, 0), (-1, -1), 3),
                ("TOPPADDING", (0, 0), (-1, -1), 3),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
            ]
            + [
                ("BACKGROUND", (0, row), (-1, row), colors.HexColor("#f2f6f7"))
                for row in range(header_rows + 1, len(data), 2)
            ]
        )
    )
    return table


def build_styles() -> dict[str, ParagraphStyle]:
    base = getSampleStyleSheet()
    return {
        "body": ParagraphStyle(
            "KoreanBody",
            parent=base["BodyText"],
            fontName="HYSMyeongJo-Medium",
            fontSize=9.1,
            leading=14.3,
            textColor=colors.HexColor("#1c2630"),
            spaceAfter=6,
        ),
        "title": ParagraphStyle(
            "KoreanTitle",
            parent=base["Title"],
            fontName="HYSMyeongJo-Medium",
            fontSize=23,
            leading=31,
            alignment=TA_CENTER,
            textColor=colors.HexColor("#123b56"),
            spaceBefore=42 * mm,
            spaceAfter=11 * mm,
        ),
        "subtitle": ParagraphStyle(
            "KoreanSubtitle",
            parent=base["Heading2"],
            fontName="HYSMyeongJo-Medium",
            fontSize=14,
            leading=21,
            alignment=TA_CENTER,
            textColor=colors.HexColor("#1d5b75"),
            spaceAfter=13 * mm,
        ),
        "h2": ParagraphStyle(
            "KoreanH2",
            parent=base["Heading2"],
            fontName="HYSMyeongJo-Medium",
            fontSize=15,
            leading=21,
            textColor=colors.HexColor("#123b56"),
            spaceBefore=10,
            spaceAfter=8,
            keepWithNext=True,
        ),
        "h3": ParagraphStyle(
            "KoreanH3",
            parent=base["Heading3"],
            fontName="HYSMyeongJo-Medium",
            fontSize=11.5,
            leading=16,
            textColor=colors.HexColor("#1d5b75"),
            spaceBefore=8,
            spaceAfter=5,
            keepWithNext=True,
        ),
        "code": ParagraphStyle(
            "Code",
            parent=base["Code"],
            fontName="HYSMyeongJo-Medium",
            fontSize=6.8,
            leading=9.5,
            backColor=colors.HexColor("#eef2f3"),
            borderColor=colors.HexColor("#c8d0d4"),
            borderWidth=0.4,
            borderPadding=6,
            spaceBefore=5,
            spaceAfter=8,
        ),
    }


def page_decoration(canvas, document) -> None:
    canvas.saveState()
    canvas.setStrokeColor(colors.HexColor("#d28b28"))
    canvas.setLineWidth(0.5)
    canvas.line(18 * mm, 13 * mm, A4[0] - 17 * mm, 13 * mm)
    canvas.setFont("HYSMyeongJo-Medium", 7.5)
    canvas.setFillColor(colors.HexColor("#5d6870"))
    canvas.drawRightString(A4[0] - 17 * mm, 8.5 * mm, str(document.page))
    canvas.restoreState()


def main() -> None:
    register_fonts()
    styles = build_styles()
    markdown = mistune.create_markdown(plugins=["table", "strikethrough"])
    soup = BeautifulSoup(markdown(SOURCE.read_text(encoding="utf-8")), "html.parser")
    story = []
    cover = True
    for node in soup.contents:
        if not isinstance(node, Tag):
            continue
        if node.name == "h1":
            story.append(paragraph_from_tag(node, styles["title"]))
        elif node.name == "h2":
            style = styles["subtitle"] if cover else styles["h2"]
            story.append(paragraph_from_tag(node, style))
            if not cover:
                story.append(HRFlowable(width="100%", thickness=1.1, color=colors.HexColor("#d28b28")))
                story.append(Spacer(1, 4))
        elif node.name == "h3":
            story.append(paragraph_from_tag(node, styles["h3"]))
        elif node.name == "p":
            image = node.find("img", recursive=False)
            if image is not None:
                story.extend([Spacer(1, 5), image_flowable(image), Spacer(1, 7)])
            else:
                story.append(paragraph_from_tag(node, styles["body"]))
        elif node.name == "table":
            story.extend([Spacer(1, 3), table_flowable(node, styles["body"]), Spacer(1, 7)])
        elif node.name in {"ul", "ol"}:
            items = [
                ListItem(paragraph_from_tag(item, styles["body"]), leftIndent=12)
                for item in node.find_all("li", recursive=False)
            ]
            story.append(
                ListFlowable(
                    items,
                    bulletType="1" if node.name == "ol" else "bullet",
                    start="1",
                    leftIndent=20,
                    bulletFontName="HYSMyeongJo-Medium",
                    bulletFontSize=8,
                    spaceAfter=5,
                )
            )
        elif node.name == "blockquote":
            quote = ParagraphStyle(
                "Quote",
                parent=styles["body"],
                leftIndent=10,
                borderColor=colors.HexColor("#d28b28"),
                borderWidth=1,
                borderPadding=6,
                backColor=colors.HexColor("#f8f3ea"),
            )
            story.append(paragraph_from_tag(node, quote))
        elif node.name == "pre":
            value = html.escape(node.get_text()).replace("\n", "<br/>")
            story.append(Paragraph(value, styles["code"]))
        elif node.name == "hr":
            if cover:
                story.append(PageBreak())
                cover = False
            else:
                story.append(HRFlowable(width="100%", thickness=0.5, color=colors.HexColor("#b8c3c9")))

    document = SimpleDocTemplate(
        str(TARGET),
        pagesize=A4,
        rightMargin=17 * mm,
        leftMargin=18 * mm,
        topMargin=16 * mm,
        bottomMargin=18 * mm,
        title="IMU 기반 상태추정과 Invariant EKF의 데이터셋별 검증",
        author="INHA-Artemis State Estimation Team",
    )
    document.build(story, onFirstPage=page_decoration, onLaterPages=page_decoration)
    print(TARGET)


if __name__ == "__main__":
    main()
