#!/usr/bin/env python3
"""Build the student handout and installation appendix; never read TA files."""
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
HW = ROOT / 'assignments/hw01'
OUT = ROOT / 'build/hw01'


def document_links(document, base):
    """PDF readers need web links; retain local image paths for embedding."""
    def rewrite(match):
        label, target = match.groups()
        if not target.startswith(('https:', 'http:', '#')):
            target = 'https://github.com/lifshitg/robotics-course-HW/blob/main/' + str(
                (base / target).resolve().relative_to(ROOT))
        return f'[{label}]({target})'
    return re.sub(r'(?<!!)\[([^\]]*)\]\(([^)]+)\)', rewrite, document)


def main():
    missing = [name for name in ['pandoc', 'xelatex'] if shutil.which(name) is None]
    if missing:
        print('Missing PDF tools: ' + ', '.join(missing), file=sys.stderr)
        print('Install: sudo apt install pandoc texlive-xetex texlive-fonts-recommended fonts-dejavu-core', file=sys.stderr)
        return 1
    OUT.mkdir(parents=True, exist_ok=True)
    text = document_links((HW / 'instructions.md').read_text(), HW)
    text = text.replace('## Interfaces', r'\newpage' + '\n\n## Interfaces')
    text += '\n\\newpage\n\n# Appendix: Installation and readiness guide\n\n'
    guide = document_links((HW / 'environment/README.md').read_text(), HW / 'environment')
    text += '\n'.join('##' + line[1:] if line.startswith('#') else line for line in guide.splitlines())
    combined = OUT / 'handout.md'
    combined.write_text(text)
    subprocess.run([
        'pandoc', str(combined), '--from=markdown', '--standalone', '--toc',
        '--pdf-engine=xelatex', '--resource-path=' + str(HW),
        '--include-in-header=' + str(ROOT / 'tools/pdf_header.tex'),
        '--lua-filter=' + str(ROOT / 'tools/pdf_code.lua'),
        '-V', 'papersize=a4', '-V', 'geometry:margin=22mm', '-V', 'fontsize=11pt',
        '-V', 'mainfont=DejaVu Serif', '-V', 'monofont=DejaVu Sans Mono',
        '-V', 'colorlinks=true', '--highlight-style=tango',
        '-o', str(OUT / 'hw01.pdf'),
    ], cwd=HW, check=True)
    print(OUT / 'hw01.pdf')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
