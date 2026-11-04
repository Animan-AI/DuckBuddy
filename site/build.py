#!/usr/bin/env python3
"""Build the DuckBuddy project page (GitHub Pages) into site/_site.

    pip install markdown==3.7 pymdown-extensions==10.16
    python site/build.py            # then open site/_site/index.html, or
    python -m http.server -d site/_site 8000

content/*.md and content/*.html are the pages: a front matter block (title, lead) and then the body,
Markdown or plain HTML. Each goes through template.html; Markdown pages get a table of contents from
their ## headings. The pictures come from docs/media, so the README and the site share one copy.
The workflow .github/workflows/pages.yml runs this on every push to main that touches either.
"""
import re
import shutil
from pathlib import Path

import markdown

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
OUT = HERE / "_site"
MEDIA = ["demo.gif", "wechat-group.png", "arch-software-zh.png", "arch-hardware-zh.png"]
# Where the system image can be downloaded; empty: the home page says to ask in the group.
IMAGE_URL = ""
NAV = [("index", "首页"), ("bom", "物料清单"), ("tutorial", "新手教程")]


def front_matter(text: str) -> tuple[dict, str]:
    m = re.match(r"^---\n(.*?)\n---\n", text, re.S)
    if not m:
        return {}, text
    meta = {}
    for line in m.group(1).splitlines():
        k, _, v = line.partition(":")
        meta[k.strip()] = v.strip()
    return meta, text[m.end():]


def render(path: Path) -> tuple[dict, str, str]:
    meta, body = front_matter(path.read_text(encoding="utf-8").replace("\r\n", "\n"))
    if path.suffix != ".md":
        return meta, body, ""
    # superfences: code blocks inside list items (a step's command), which fenced_code does not do
    md = markdown.Markdown(extensions=["tables", "pymdownx.superfences", "attr_list", "toc", "sane_lists"],
                           extension_configs={"toc": {"toc_depth": "2-2"}})
    html = md.convert(body)
    # tables scroll sideways on a phone instead of widening the page
    html = html.replace("<table>", '<div class="table-wrap"><table>').replace("</table>", "</table></div>")
    items = "".join(f'<li><a href="#{t["id"]}">{t["name"]}</a></li>'
                    for h1 in md.toc_tokens for t in (h1.get("children") or [h1]) if t["level"] == 2)
    toc = f'<nav class="toc" aria-label="目录"><p>目录</p><ol>{items}</ol></nav>' if items else ""
    return meta, html, toc


def main():
    if OUT.exists():
        shutil.rmtree(OUT)
    (OUT / "media").mkdir(parents=True)
    template = (HERE / "template.html").read_text(encoding="utf-8")
    image_link = (f'<a class="btn primary" href="{IMAGE_URL}">下载镜像</a>' if IMAGE_URL
                  else "下载链接整理中，暂时请到下面的交流群里获取。")
    for page in sorted((HERE / "content").glob("*.*")):
        meta, body, toc = render(page)
        name = page.stem
        current = ' aria-current="page"'
        nav = "".join(f'<a href="{n}.html"{current if n == name else ""}>{label}</a>' for n, label in NAV)
        title = meta.get("title", "DuckBuddy")
        html = (template.replace("{{TITLE}}", title if name == "index" else f"{title} · DuckBuddy")
                .replace("{{DESCRIPTION}}", meta.get("lead", ""))
                .replace("{{NAV}}", nav)
                .replace("{{MAIN_CLASS}}", "home" if meta.get("home") else ("doc has-toc" if toc else "doc"))
                .replace("{{TOC}}", toc)
                .replace("{{BODY}}", body)
                .replace("{{IMAGE_LINK}}", image_link))
        (OUT / f"{name}.html").write_text(html, encoding="utf-8")
    for f in ["style.css", "favicon.svg"]:
        shutil.copy2(HERE / f, OUT / f)
    for f in MEDIA:
        shutil.copy2(REPO / "docs" / "media" / f, OUT / "media" / f)
    (OUT / ".nojekyll").write_text("")
    print(f"built {OUT}")


if __name__ == "__main__":
    main()
