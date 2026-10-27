#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Generate an offline reference from the public header contracts."""
from html import escape
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
PATTERN = re.compile(r'/\*\*((?:(?!\*/).)*)\*/\s*([\w\s*]+\briptide_\w+\s*\([^;]*?\);)', re.S)

def comment_sections(raw):
    lines = [re.sub(r'^\s*\* ?', '', line).strip() for line in raw.splitlines()]
    sections = [('description', '', [])]
    for line in lines:
        tag = re.match(r'@param\s+(\w+)\s*(.*)', line)
        if tag:
            sections.append(('param', tag[1], [tag[2]]))
        elif line.startswith('@return'):
            sections.append(('return', '', [line[len('@return'):].strip()]))
        elif line:
            sections[-1][2].append(line)
    return [(kind, name, ' '.join(parts)) for kind, name, parts in sections]

def main():
    groups = []
    documented = set()
    for header in sorted((ROOT / 'include/riptide').glob('*.h')):
        functions = []
        source = header.read_text()
        for match in PATTERN.finditer(source):
            comment, prototype = match.groups()
            name = re.search(r'\b(riptide_\w+)\s*\(', prototype)[1]
            sections = comment_sections(comment)
            # Keep documentation mechanically aligned with the C signature.
            arguments = prototype[prototype.index('(')+1:prototype.rindex(')')].strip()
            parameters = [] if arguments == 'void' else [re.findall(r'\w+', arg)[-1] for arg in arguments.split(',')]
            described = [param for kind, param, _ in sections if kind == 'param']
            if parameters != described:
                raise SystemExit(f'{name}: parameters {parameters} differ from comments {described}')
            body = []
            rows = []
            for kind, param, text in sections:
                if kind == 'description':
                    body.append(f'<p>{escape(text)}</p>')
                elif kind == 'param':
                    rows.append(f'<tr><th scope="row"><code>{escape(param)}</code></th><td>{escape(text)}</td></tr>')
                else:
                    body.append(f'<p class="returns"><strong>Returns</strong> {escape(text)}</p>')
            params = '<table><caption>Parameters</caption><tbody>' + ''.join(rows) + '</tbody></table>' if rows else ''
            description = body[0] if body else ''
            returns = ''.join(body[1:])
            functions.append((name, f'<article id="{name}"><h3>{name}</h3>{description}<pre><code>{escape(prototype.strip())}</code></pre>{params}{returns}</article>'))
            documented.add(name)
        declared = set(re.findall(r'\b(riptide_\w+)\s*\([^;{{}}]*\)\s*;', source, re.S))
        missing = declared - documented
        if missing:
            raise SystemExit(f'Undocumented public functions in {header.name}: {sorted(missing)}')
        if functions:
            groups.append((header.stem, functions))
    nav = ''.join(f'<details open><summary>{escape(group)}</summary>' + ''.join(f'<a href="#{name}">{name}</a>' for name, _ in functions) + '</details>' for group, functions in groups)
    content = ''.join(f'<section><h2>{escape(group)}.h</h2>' + ''.join(body for _, body in functions) + '</section>' for group, functions in groups)
    page = '''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Riptide Engine — API reference</title><style>
:root{color-scheme:dark;--bg:#10171d;--panel:#17222c;--line:#334652;--ink:#e1eaf0;--muted:#a8bac6;--accent:#73dac4}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:16px/1.65 system-ui,sans-serif}header{padding:32px 40px;border-bottom:1px solid var(--line)}header h1{margin:0;font-size:32px}header p{margin:8px 0 0;color:var(--muted)}a{color:var(--accent)}.layout{display:grid;grid-template-columns:300px minmax(0,1fr);max-width:1500px;margin:auto}nav{padding:24px;position:sticky;top:0;max-height:100vh;overflow:auto;font-size:13px}nav a{display:block;padding:3px 0;text-decoration:none}summary{font-weight:700;margin:14px 0 6px;font-size:16px}main{padding:0 36px 60px;max-width:1100px}h2{color:var(--accent);margin:36px 0 12px;font-size:26px}article{scroll-margin-top:20px;margin:0 0 24px;padding:24px;background:var(--panel);border:1px solid var(--line);border-radius:8px}h3{font-size:19px;margin:0 0 12px;overflow-wrap:anywhere}p{margin:10px 0}pre{white-space:pre-wrap;overflow-wrap:anywhere;padding:16px;background:#0c131a;border-radius:5px;font-size:13px;line-height:1.6}code{font-family:ui-monospace,Consolas,monospace}table{border-collapse:collapse;width:100%;font-size:14px}caption{text-align:left;font-weight:700;margin:12px 0 6px}th,td{text-align:left;vertical-align:top;padding:9px 12px 9px 0;border-bottom:1px solid var(--line)}th{width:125px;color:var(--accent)}.returns{padding-top:10px;color:var(--muted)}@media(max-width:800px){.layout{display:block}nav{position:static;max-height:250px;border-bottom:1px solid var(--line)}header{padding:24px}main{padding:0 16px 40px}article{padding:18px}}
</style></head><body><header><h1>Riptide Engine <span style="color:var(--accent)">0.1.0</span></h1><p>Public C API · purpose, parameters, return values and lifetime contracts</p><p><a href="../README.md">Readme</a> · <a href="API.md">Conventions</a> · <a href="INTEGRATION.md">Integration</a> · <a href="GPU.md">GPU backends</a></p></header><div class="layout"><nav aria-label="Function index">'''+nav+'</nav><main>'+content+'</main></div></body></html>'
    (ROOT / 'docs/reference.html').write_text(page)
    print(f'Documented {len(documented)} public functions; all parameter names match.')

if __name__ == '__main__':
    main()
