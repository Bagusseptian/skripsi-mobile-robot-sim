"""
Ubah labsheet Markdown (berikut gambar) menjadi dokumen Word (.docx) siap cetak.

Pemakaian:
    python buat_docx_labsheet.py Labsheet_0_Jembatan_OpenCV_ke_Kamera_RGBD.md

Mendukung subset Markdown yang dipakai labsheet: judul (#, ##, ###), paragraf,
**tebal**, *miring*, `kode`, $rumus$ sederhana, daftar bernomor/berpoin, tabel,
blok kode ```, kutipan >, gambar ![keterangan](path) dan rumus blok $$...$$.
"""
import io
import os
import re
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from docx import Document
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

FONT = 'Times New Roman'
MONO = 'Consolas'


def shading(el, warna):
    pr = el.get_or_add_pPr() if hasattr(el, 'get_or_add_pPr') else el.get_or_add_tcPr()
    shd = OxmlElement('w:shd')
    shd.set(qn('w:val'), 'clear')
    shd.set(qn('w:color'), 'auto')
    shd.set(qn('w:fill'), warna)
    pr.append(shd)


def border_kiri(p, warna='808080'):
    pPr = p._p.get_or_add_pPr()
    b = OxmlElement('w:pBdr')
    left = OxmlElement('w:left')
    for k, v in (('w:val', 'single'), ('w:sz', '18'), ('w:space', '6'), ('w:color', warna)):
        left.set(qn(k), v)
    b.append(left)
    pPr.append(b)


def math_runs(p, teks, size=None):
    """Rumus inline sederhana: f_x -> f dengan subskrip x."""
    teks = teks.replace('\\,', ' ').replace('{,}', ',').replace('\\times', '×')
    for bagian in re.split(r'([A-Za-z])_\{?([A-Za-z0-9]+)\}?', teks):
        pass
    pos = 0
    for m in re.finditer(r'([A-Za-z])_\{?([A-Za-z0-9]+)\}?', teks):
        if m.start() > pos:
            r = p.add_run(teks[pos:m.start()])
            r.italic = True
        r = p.add_run(m.group(1))
        r.italic = True
        s = p.add_run(m.group(2))
        s.italic = True
        s.font.subscript = True
        pos = m.end()
    if pos < len(teks):
        r = p.add_run(teks[pos:])
        r.italic = True


def inline(p, teks, bold=False, size=None):
    """Tambahkan teks dengan format inline Markdown ke paragraf."""
    pola = re.compile(r'(\*\*.+?\*\*|`[^`]+`|\$[^$]+\$|\*[^*]+\*)')
    for bagian in pola.split(teks):
        if not bagian:
            continue
        if bagian.startswith('**') and bagian.endswith('**'):
            isi = bagian[2:-2]
            # miring di dalam tebal
            for sub in re.split(r'(\*[^*]+\*)', isi):
                if not sub:
                    continue
                r = p.add_run(sub.strip('*') if sub.startswith('*') else sub)
                r.bold = True
                r.italic = sub.startswith('*')
        elif bagian.startswith('`'):
            r = p.add_run(bagian[1:-1])
            r.font.name = MONO
            r.font.size = Pt(10)
            r.font.color.rgb = RGBColor(0xA0, 0x1E, 0x1E)
        elif bagian.startswith('$'):
            math_runs(p, bagian[1:-1])
        elif bagian.startswith('*'):
            r = p.add_run(bagian[1:-1])
            r.italic = True
            r.bold = bold
        else:
            r = p.add_run(bagian)
            r.bold = bold
        if size:
            for r in p.runs:
                r.font.size = Pt(size)


def rumus_gambar(latex):
    """Render rumus blok LaTeX menjadi gambar PNG (mathtext matplotlib)."""
    latex = latex.replace('\\qquad', r'\qquad')
    fig = plt.figure(figsize=(0.01, 0.01), dpi=300)
    fig.text(0, 0, f'${latex}$', fontsize=15)
    buf = io.BytesIO()
    fig.savefig(buf, format='png', bbox_inches='tight', pad_inches=0.05, transparent=False, facecolor='white')
    plt.close(fig)
    buf.seek(0)
    return buf


class Pembangun:
    def __init__(self, md_path):
        self.md_path = md_path
        self.dir = os.path.dirname(os.path.abspath(md_path))
        self.doc = Document()
        self.no_gambar = 0
        self._gaya()

    def _gaya(self):
        sec = self.doc.sections[0]
        sec.page_width, sec.page_height = Cm(21.0), Cm(29.7)
        for side in ('left_margin', 'right_margin', 'top_margin', 'bottom_margin'):
            setattr(sec, side, Cm(2.5))
        st = self.doc.styles['Normal']
        st.font.name = FONT
        st.font.size = Pt(12)
        st.element.rPr.rFonts.set(qn('w:eastAsia'), FONT)
        st.paragraph_format.space_after = Pt(6)
        st.paragraph_format.line_spacing = 1.15
        for nama, ukuran, sebelum in (('Title', 15, 0), ('Heading 1', 13, 14), ('Heading 2', 12, 10), ('Heading 3', 12, 8)):
            s = self.doc.styles[nama]
            s.font.name = FONT
            s.font.size = Pt(ukuran)
            s.font.bold = True
            s.font.color.rgb = RGBColor(0, 0, 0)
            s.element.rPr.rFonts.set(qn('w:eastAsia'), FONT)
            s.element.rPr.rFonts.set(qn('w:ascii'), FONT)
            s.element.rPr.rFonts.set(qn('w:hAnsi'), FONT)
            s.paragraph_format.space_before = Pt(sebelum)
            s.paragraph_format.space_after = Pt(6)
            s.paragraph_format.keep_with_next = True
            rf = s.element.rPr.rFonts
            for a in ('w:asciiTheme', 'w:hAnsiTheme', 'w:eastAsiaTheme', 'w:cstheme'):
                if rf.get(qn(a)) is not None:
                    del rf.attrib[qn(a)]
        # nomor halaman di footer
        p = sec.footer.paragraphs[0]
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        r = p.add_run()
        for tag, teks in (('begin', None), (None, 'PAGE'), ('end', None)):
            if tag:
                f = OxmlElement('w:fldChar')
                f.set(qn('w:fldCharType'), tag)
                r._r.append(f)
            else:
                t = OxmlElement('w:instrText')
                t.set(qn('xml:space'), 'preserve')
                t.text = teks
                r._r.append(t)

    def gambar(self, path, keterangan):
        full = os.path.join(self.dir, path)
        terminal = os.path.basename(path).startswith('tm')
        lebar = Cm(15.0) if terminal else Cm(14.5)
        if 'denah' in path:
            lebar = Cm(12.0)
        if 'gb04' in path or 'gb05' in path or 'gb09' in path:
            lebar = Cm(11.0)
        p = self.doc.add_paragraph()
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        p.paragraph_format.keep_with_next = True
        p.add_run().add_picture(full, width=lebar)
        if keterangan.startswith('Gambar'):
            c = self.doc.add_paragraph()
            c.alignment = WD_ALIGN_PARAGRAPH.CENTER
            r = c.add_run(keterangan)
            r.font.size = Pt(10.5)
            r.bold = True

    def tabel(self, baris):
        sel = [[c.strip() for c in b.strip().strip('|').split('|')] for b in baris]
        sel = [s for s in sel if not all(re.fullmatch(r':?-+:?', c) for c in s)]
        ratakan = [c.strip() for c in baris[1].strip().strip('|').split('|')]
        n = len(sel[0])
        t = self.doc.add_table(rows=len(sel), cols=n)
        t.style = 'Table Grid'
        t.alignment = WD_TABLE_ALIGNMENT.CENTER
        for i, s in enumerate(sel):
            for j in range(n):
                cell = t.cell(i, j)
                p = cell.paragraphs[0]
                p.paragraph_format.space_after = Pt(0)
                teks = s[j] if j < len(s) else ''
                if j < len(ratakan) and ratakan[j].startswith(':') and ratakan[j].endswith(':'):
                    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
                inline(p, teks, bold=(i == 0), size=10.5)
                if i < len(sel) - 1:
                    p.paragraph_format.keep_with_next = True
                trPr = t.rows[i]._tr.get_or_add_trPr()
                if trPr.find(qn('w:cantSplit')) is None:
                    trPr.append(OxmlElement('w:cantSplit'))
                if i == 0:
                    shading(cell._tc, 'D9E2F3')
                elif not teks:
                    p.paragraph_format.space_after = Pt(12)  # ruang menulis
        self.doc.add_paragraph().paragraph_format.space_after = Pt(2)

    def kode(self, baris):
        for b in baris:
            p = self.doc.add_paragraph()
            pf = p.paragraph_format
            pf.space_after = Pt(0)
            pf.left_indent = Cm(0.5)
            pf.line_spacing = 1.0
            shading(p._p, 'F2F2F2')
            r = p.add_run(b if b else ' ')
            r.font.name = MONO
            r.font.size = Pt(9.5)
        self.doc.add_paragraph().paragraph_format.space_after = Pt(0)

    def bangun(self, keluaran):
        baris = open(self.md_path, encoding='utf-8').read().split('\n')
        i = 0
        while i < len(baris):
            b = baris[i]
            s = b.strip()
            if s.startswith('```'):
                isi = []
                i += 1
                while not baris[i].strip().startswith('```'):
                    isi.append(baris[i].strip())
                    i += 1
                self.kode(isi)
            elif s.startswith('$$'):
                latex = s.strip('$')
                p = self.doc.add_paragraph()
                p.alignment = WD_ALIGN_PARAGRAPH.CENTER
                p.add_run().add_picture(rumus_gambar(latex), height=Cm(1.3))
            elif s.startswith('|'):
                blok = []
                while i < len(baris) and baris[i].strip().startswith('|'):
                    blok.append(baris[i])
                    i += 1
                self.tabel(blok)
                continue
            elif re.match(r'!\[(.*)\]\((.*)\)', s):
                m = re.match(r'!\[(.*)\]\((.*)\)', s)
                self.gambar(m.group(2), m.group(1))
            elif s.startswith('# '):
                p = self.doc.add_paragraph(style='Title')
                p.alignment = WD_ALIGN_PARAGRAPH.CENTER
                inline(p, s[2:], bold=True)
            elif s.startswith('## '):
                inline(self.doc.add_paragraph(style='Heading 1'), s[3:], bold=True)
            elif s.startswith('### '):
                inline(self.doc.add_paragraph(style='Heading 2'), s[4:], bold=True)
            elif s.startswith('>'):
                isi = []
                while i < len(baris) and baris[i].strip().startswith('>'):
                    t = baris[i].strip()[1:].strip()
                    if t:
                        isi.append(t)
                    i += 1
                for t in isi:
                    p = self.doc.add_paragraph()
                    p.paragraph_format.left_indent = Cm(0.4)
                    p.paragraph_format.space_after = Pt(2)
                    border_kiri(p)
                    inline(p, t, size=10.5)
                continue
            elif s == '---' or not s:
                pass
            elif re.match(r'\d+\.\s', s):
                p = self.doc.add_paragraph(style='List Number' if not b.startswith(' ') else 'List Number 2')
                inline(p, re.sub(r'^\d+\.\s+', '', s))
            elif s.startswith('- '):
                p = self.doc.add_paragraph(style='List Bullet' if not b.startswith(' ') else 'List Bullet 2')
                inline(p, s[2:])
            else:
                p = self.doc.add_paragraph()
                p.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
                inline(p, s.rstrip())
            i += 1
        self._restart_penomoran()
        self.doc.save(keluaran)

    def _restart_penomoran(self):
        """Mulai ulang daftar bernomor setiap kali ada judul atau elemen lain di antaranya."""
        body = self.doc.element.body
        numbering = self.doc.part.numbering_part.numbering_definitions._numbering
        num_id_list = None
        st = self.doc.styles['List Number']
        num_pr = st.element.pPr.numPr if st.element.pPr is not None else None
        if num_pr is None:
            return
        abstract = numbering.num_having_numId(num_pr.numId.val).abstractNumId.val
        sebelumnya_daftar = False
        for el in body.iterchildren():
            gaya = None
            if el.tag == qn('w:p') and el.pPr is not None and el.pPr.pStyle is not None:
                gaya = el.pPr.pStyle.val
            if gaya == 'ListNumber':
                if not sebelumnya_daftar:
                    num = numbering.add_num(abstract)
                    lvl = num.add_lvlOverride(ilvl=0)
                    lvl.add_startOverride(1)
                    num_id_list = num.numId
                pPr = el.get_or_add_pPr()
                np_ = pPr.get_or_add_numPr()
                np_.get_or_add_ilvl().val = 0
                np_.get_or_add_numId().val = num_id_list
                sebelumnya_daftar = True
            elif gaya in ('ListNumber2', 'ListBullet', 'ListBullet2'):
                continue
            elif gaya in ('Title', 'Heading1', 'Heading2', 'Heading3'):
                sebelumnya_daftar = False


if __name__ == '__main__':
    md = sys.argv[1]
    out = os.path.splitext(md)[0] + '.docx'
    Pembangun(md).bangun(out)
    print('Dokumen dibuat:', out)
