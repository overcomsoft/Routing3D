# Markdown → PDF 변환기 (의존성 0 · Markdown→HTML→Word COM PDF)
# =============================================================================
# [이 스크립트가 하는 일]
#   임의의 Markdown(.md) 문서를 표준 라이브러리만으로 HTML 로 변환한 뒤, 설치된
#   MS Word 를 COM 으로 띄워 그 HTML 을 열고 PDF 로 저장한다(pandoc/markdown 패키지 불필요).
#   docs/ 의 손으로 쓴 markdown 문서(예: routing3d_group_pattern_routing.md)를 PDF 로 뽑는 용도.
#
#   지원 문법: ATX 헤더(#~######) · 표(GFM 파이프) · 펜스 코드블록(```) · 인라인 코드(`x`) ·
#             굵게(**x**) · 링크([t](u)) · 순서/비순서 리스트 · blockquote(>) · 수평선(---).
#   한글: HTML 을 UTF-8(BOM) 로 쓰고 <meta charset> 지정 → Word 가 인코딩 자동 인식.
#
# [실행]  (프로젝트 루트에서)
#   .\.venv\Scripts\python.exe python_experiments/out/_md_to_pdf.py ^
#       docs/routing3d_group_pattern_routing.md docs/routing3d_group_pattern_routing.pdf
#   (출력 인자 생략 시 입력과 같은 경로에 .pdf 로 저장)
# =============================================================================
from __future__ import annotations
import html
import os
import re
import sys
import tempfile


# ---- 인라인 변환: 굵게 · 인라인코드 · 링크 → HTML (먼저 escape 후 토큰 치환) ----
def _inline(text: str) -> str:
    # 코드 스팬을 먼저 자리표시자로 빼서 내부가 다른 규칙에 안 먹히게 한다.
    spans: list[str] = []

    def _stash_code(m: re.Match) -> str:
        spans.append(html.escape(m.group(1)))
        return f"\x00{len(spans) - 1}\x00"

    text = re.sub(r"`([^`]+)`", _stash_code, text)
    text = html.escape(text)
    # 링크 [t](u) — escape 된 뒤이므로 대괄호/괄호는 그대로 남아있다.
    text = re.sub(r"\[([^\]]+)\]\(([^)]+)\)",
                  lambda m: f'<a href="{m.group(2)}">{m.group(1)}</a>', text)
    # 굵게 **x**
    text = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", text)
    # 코드 스팬 복원
    text = re.sub(r"\x00(\d+)\x00", lambda m: f"<code>{spans[int(m.group(1))]}</code>", text)
    return text


def _cells(line: str) -> list[str]:
    s = line.strip()
    if s.startswith("|"):
        s = s[1:]
    if s.endswith("|"):
        s = s[:-1]
    return [c.strip() for c in s.split("|")]


def md_to_html(md: str) -> str:
    lines = md.replace("\r\n", "\n").split("\n")
    out: list[str] = []
    i = 0
    n = len(lines)
    list_stack: list[str] = []   # 'ul' / 'ol' 중첩(단순: 한 단계만)

    def close_lists():
        while list_stack:
            out.append(f"</{list_stack.pop()}>")

    while i < n:
        line = lines[i]

        # 펜스 코드블록
        m = re.match(r"^\s*```(.*)$", line)
        if m:
            close_lists()
            i += 1
            buf: list[str] = []
            while i < n and not re.match(r"^\s*```\s*$", lines[i]):
                buf.append(lines[i])
                i += 1
            i += 1  # 닫는 펜스 소비
            out.append("<pre><code>" + html.escape("\n".join(buf)) + "</code></pre>")
            continue

        # 표: 헤더줄 + 구분줄(---|---)
        if "|" in line and i + 1 < n and re.match(r"^\s*\|?[\s:_-]*\|[\s:|_-]*$", lines[i + 1]) \
                and "|" in lines[i + 1]:
            close_lists()
            header = _cells(line)
            i += 2  # 헤더 + 구분
            rows: list[list[str]] = []
            while i < n and "|" in lines[i] and lines[i].strip():
                rows.append(_cells(lines[i]))
                i += 1
            t = ["<table>", "<thead><tr>"]
            t += [f"<th>{_inline(c)}</th>" for c in header]
            t.append("</tr></thead><tbody>")
            for r in rows:
                t.append("<tr>" + "".join(f"<td>{_inline(c)}</td>" for c in r) + "</tr>")
            t.append("</tbody></table>")
            out.append("".join(t))
            continue

        # 수평선
        if re.match(r"^\s*---+\s*$", line):
            close_lists()
            out.append("<hr/>")
            i += 1
            continue

        # 헤더
        m = re.match(r"^(#{1,6})\s+(.*)$", line)
        if m:
            close_lists()
            lvl = len(m.group(1))
            out.append(f"<h{lvl}>{_inline(m.group(2).strip())}</h{lvl}>")
            i += 1
            continue

        # blockquote (연속 줄 묶기)
        if re.match(r"^\s*>\s?", line):
            close_lists()
            buf = []
            while i < n and re.match(r"^\s*>\s?", lines[i]):
                buf.append(_inline(re.sub(r"^\s*>\s?", "", lines[i])))
                i += 1
            out.append("<blockquote>" + "<br/>".join(buf) + "</blockquote>")
            continue

        # 순서 리스트
        m = re.match(r"^\s*\d+\.\s+(.*)$", line)
        if m:
            if list_stack[-1:] != ["ol"]:
                close_lists()
                list_stack.append("ol")
                out.append("<ol>")
            out.append(f"<li>{_inline(m.group(1))}</li>")
            i += 1
            continue

        # 비순서 리스트
        m = re.match(r"^\s*[-*+]\s+(.*)$", line)
        if m:
            if list_stack[-1:] != ["ul"]:
                close_lists()
                list_stack.append("ul")
                out.append("<ul>")
            out.append(f"<li>{_inline(m.group(1))}</li>")
            i += 1
            continue

        # 빈 줄
        if not line.strip():
            close_lists()
            i += 1
            continue

        # 일반 문단
        close_lists()
        out.append(f"<p>{_inline(line.strip())}</p>")
        i += 1

    close_lists()
    return "\n".join(out)


_CSS = """
body { font-family: 'Malgun Gothic','맑은 고딕',sans-serif; font-size: 10.5pt;
       line-height: 1.5; color: #1a1a1a; }
h1 { font-size: 19pt; border-bottom: 2px solid #385b85; padding-bottom: 4px; color:#1c2b45; }
h2 { font-size: 15pt; border-bottom: 1px solid #c5cee0; padding-bottom: 3px; color:#23406a; margin-top: 18px; }
h3 { font-size: 12.5pt; color:#2b4a78; margin-top: 14px; }
h4 { font-size: 11pt; color:#33507a; }
code { font-family: 'Consolas','D2Coding',monospace; background:#f0f2f7; padding:1px 4px;
       border-radius:3px; font-size: 9.5pt; color:#9c2d40; }
pre { background:#1e2230; color:#e8eaf0; padding:10px 12px; border-radius:5px;
      font-family:'Consolas',monospace; font-size:9pt; line-height:1.35; overflow:auto; }
pre code { background:transparent; color:#e8eaf0; padding:0; }
table { border-collapse: collapse; width:100%; margin:8px 0; font-size:9.5pt; }
th,td { border:1px solid #b8c2d8; padding:5px 8px; text-align:left; vertical-align:top; }
th { background:#2b3548; color:#e8eaf0; }
tr:nth-child(even) td { background:#f4f6fb; }
blockquote { border-left:4px solid #385b85; margin:8px 0; padding:4px 14px;
             background:#f5f8fc; color:#3a4862; }
a { color:#2b5fa5; text-decoration:none; }
hr { border:none; border-top:1px solid #d0d7e6; margin:16px 0; }
"""


def build_html(md: str, title: str) -> str:
    body = md_to_html(md)
    return (f"<!DOCTYPE html><html><head><meta charset='utf-8'>"
            f"<title>{html.escape(title)}</title><style>{_CSS}</style></head>"
            f"<body>{body}</body></html>")


def html_to_pdf_word(html_path: str, pdf_path: str) -> None:
    import win32com.client  # pywin32
    word = win32com.client.DispatchEx("Word.Application")
    word.Visible = False
    try:
        doc = word.Documents.Open(html_path, False, True)
        doc.SaveAs(pdf_path, FileFormat=17)  # wdFormatPDF
        doc.Close(False)
    finally:
        word.Quit()


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: _md_to_pdf.py <in.md> [out.pdf]")
        return 2
    md_path = os.path.abspath(sys.argv[1])
    pdf_path = os.path.abspath(sys.argv[2]) if len(sys.argv) > 2 \
        else os.path.splitext(md_path)[0] + ".pdf"
    with open(md_path, encoding="utf-8") as f:
        md = f.read()
    title = os.path.splitext(os.path.basename(md_path))[0]
    html_doc = build_html(md, title)

    # Word 가 인코딩을 확실히 인식하도록 UTF-8 BOM 로 임시 HTML 저장.
    fd, html_path = tempfile.mkstemp(suffix=".html")
    os.close(fd)
    with open(html_path, "w", encoding="utf-8-sig") as f:
        f.write(html_doc)
    try:
        html_to_pdf_word(html_path, pdf_path)
        print(f"[pdf] 저장: {pdf_path}")
    finally:
        try:
            os.remove(html_path)
        except OSError:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
