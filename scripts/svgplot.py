"""Minimal deterministic SVG plotting (standard library) for I1 figures.

Okabe-Ito colour-blind-safe palette. Every figure carries <title> and <desc>.
Number formatting is fixed so that output bytes are reproducible.
"""

from __future__ import annotations

import html
import math

PALETTE = {
    "black": "#222222", "orange": "#E69F00", "sky": "#56B4E9", "green": "#009E73",
    "yellow": "#F0E442", "blue": "#0072B2", "vermillion": "#D55E00", "purple": "#CC79A7",
    "grey": "#8C8C8C", "grid": "#E3E6EA", "text": "#1F2328",
}
PHASE_FILL = {
    "quiet_stance": "#F2F2F2", "static_hold": "#F2F2F2", "unweighting": "#DCEBF7", "braking": "#FBE3CF",
    "propulsion": "#DDF1E4", "flight": "#FFF6CC",
}


def _f(v: float) -> str:
    return f"{v:.2f}"


def nice_ticks(lo: float, hi: float, n: int = 5) -> list[float]:
    if hi <= lo:
        hi = lo + 1.0
    raw = (hi - lo) / n
    mag = 10 ** math.floor(math.log10(raw))
    step = min((m * mag for m in (1, 2, 2.5, 5, 10) if m * mag >= raw), default=10 * mag)
    start = math.ceil(lo / step) * step
    ticks = []
    v = start
    while v <= hi + 1e-12 * abs(hi):
        ticks.append(round(v, 10))
        v += step
    return ticks


class Panel:
    def __init__(self, fig, x, y, w, h, xr, yr, title="", xlabel="", ylabel=""):
        self.fig, self.x, self.y, self.w, self.h = fig, x, y, w, h
        self.xr, self.yr = xr, yr
        self.title, self.xlabel, self.ylabel = title, xlabel, ylabel
        self.items: list[str] = []
        self.legend: list[tuple[str, str, str]] = []
        self.xtick_labels: dict[float, str] | None = None  # category axis when set
        self.show_yticks = True
        self.legend_top = False

    def px(self, v):
        return self.x + (v - self.xr[0]) / (self.xr[1] - self.xr[0]) * self.w

    def py(self, v):
        return self.y + self.h - (v - self.yr[0]) / (self.yr[1] - self.yr[0]) * self.h

    def band(self, x0, x1, color, label=None):
        x0, x1 = max(x0, self.xr[0]), min(x1, self.xr[1])
        if x1 <= x0:
            return
        self.items.append(
            f'<rect x="{_f(self.px(x0))}" y="{_f(self.y)}" width="{_f(self.px(x1) - self.px(x0))}" '
            f'height="{_f(self.h)}" fill="{color}"/>'
        )
        if label:
            self.items.append(
                f'<text x="{_f((self.px(x0) + self.px(x1)) / 2)}" y="{_f(self.y + 11)}" font-size="9" '
                f'text-anchor="middle" fill="{PALETTE["grey"]}">{html.escape(label)}</text>'
            )

    def line(self, xs, ys, color, width=1.6, dash=None, label=None):
        pts = []
        segments = []
        for x, y in zip(xs, ys):
            if y is None or x is None or not (math.isfinite(x) and math.isfinite(y)):
                if pts:
                    segments.append(pts)
                pts = []
                continue
            pts.append(f"{_f(self.px(x))},{_f(self.py(y))}")
        if pts:
            segments.append(pts)
        d = f' stroke-dasharray="{dash}"' if dash else ""
        for seg in segments:
            self.items.append(
                f'<polyline points="{" ".join(seg)}" fill="none" stroke="{color}" stroke-width="{width}"{d} '
                f'stroke-linejoin="round"/>'
            )
        if label:
            self.legend.append(("line", color, label) if not dash else ("dash", color, label))

    def markers(self, xs, ys, color, r=1.6, label=None, hollow=False):
        for x, y in zip(xs, ys):
            if y is None:
                continue
            fill = "none" if hollow else color
            self.items.append(
                f'<circle cx="{_f(self.px(x))}" cy="{_f(self.py(y))}" r="{r}" '
                f'fill="{fill}" stroke="{color}" stroke-width="0.8"/>'
            )
        if label:
            self.legend.append(("marker", color, label))

    def missing(self, xs, color, label=None):
        for x in xs:
            self.items.append(
                f'<line x1="{_f(self.px(x))}" y1="{_f(self.y + self.h - 8)}" x2="{_f(self.px(x))}" '
                f'y2="{_f(self.y + self.h)}" stroke="{color}" stroke-width="2"/>'
            )
        if label:
            self.legend.append(("tick", color, label))

    def vline(self, x, color, label=None, dash="4 3", position=0):
        if not (self.xr[0] <= x <= self.xr[1]):
            return
        self.items.append(
            f'<line x1="{_f(self.px(x))}" y1="{_f(self.y)}" x2="{_f(self.px(x))}" y2="{_f(self.y + self.h)}" '
            f'stroke="{color}" stroke-width="1.1" stroke-dasharray="{dash}"/>'
        )
        if label:
            self.items.append(
                f'<text x="{_f(self.px(x) + 3)}" y="{_f(self.y + 24 + 11 * position)}" font-size="9" '
                f'fill="{color}">{html.escape(label)}</text>'
            )

    def hline(self, y, color, label=None, dash="2 3"):
        if not (self.yr[0] <= y <= self.yr[1]):
            return
        self.items.append(
            f'<line x1="{_f(self.x)}" y1="{_f(self.py(y))}" x2="{_f(self.x + self.w)}" y2="{_f(self.py(y))}" '
            f'stroke="{color}" stroke-width="1" stroke-dasharray="{dash}"/>'
        )
        if label:
            self.items.append(
                f'<text x="{_f(self.x + self.w - 4)}" y="{_f(self.py(y) - 3)}" font-size="9" text-anchor="end" '
                f'fill="{color}">{html.escape(label)}</text>'
            )

    def text(self, x, y, s, size=9, color=None, anchor="start"):
        self.items.append(
            f'<text x="{_f(self.px(x))}" y="{_f(self.py(y))}" font-size="{size}" text-anchor="{anchor}" '
            f'fill="{color or PALETTE["text"]}">{html.escape(s)}</text>'
        )

    def raw(self, s):
        self.items.append(s)

    def render(self) -> str:
        out = [f'<g><rect x="{_f(self.x)}" y="{_f(self.y)}" width="{_f(self.w)}" height="{_f(self.h)}" '
               f'fill="#FFFFFF" stroke="#C9CED6"/>']
        clip = f"clip{len(self.fig.panels)}_{int(self.x)}_{int(self.y)}"
        out.append(f'<clipPath id="{clip}"><rect x="{_f(self.x)}" y="{_f(self.y)}" width="{_f(self.w)}" '
                   f'height="{_f(self.h)}"/></clipPath>')
        for t in (nice_ticks(*self.yr) if self.show_yticks else []):
            out.append(f'<line x1="{_f(self.x)}" y1="{_f(self.py(t))}" x2="{_f(self.x + self.w)}" '
                       f'y2="{_f(self.py(t))}" stroke="{PALETTE["grid"]}" stroke-width="0.8"/>')
            out.append(f'<text x="{_f(self.x - 4)}" y="{_f(self.py(t) + 3)}" font-size="9" text-anchor="end" '
                       f'fill="{PALETTE["text"]}">{f"{t:g}".replace("-", "−")}</text>')
        xticks = self.xtick_labels or {t: f"{t:g}".replace("-", "−") for t in nice_ticks(*self.xr)}
        for t, label in xticks.items():
            out.append(f'<text x="{_f(self.px(t))}" y="{_f(self.y + self.h + 12)}" font-size="9" '
                       f'text-anchor="middle" fill="{PALETTE["text"]}">{html.escape(label)}</text>')
        out.append(f'<g clip-path="url(#{clip})">' + "".join(self.items) + "</g>")
        out.append(f'<text x="{_f(self.x)}" y="{_f(self.y - 6)}" font-size="11" font-weight="bold" '
                   f'fill="{PALETTE["text"]}">{html.escape(self.title)}</text>')
        out.append(f'<text x="{_f(self.x + self.w / 2)}" y="{_f(self.y + self.h + 25)}" font-size="9.5" '
                   f'text-anchor="middle" fill="{PALETTE["text"]}">{html.escape(self.xlabel)}</text>')
        out.append(f'<text transform="translate({_f(self.x - 34)},{_f(self.y + self.h / 2)}) rotate(-90)" '
                   f'font-size="9.5" text-anchor="middle" fill="{PALETTE["text"]}">{html.escape(self.ylabel)}</text>')
        lx, ly = self.x + 6, self.y + self.h - 8 - 11 * (len(self.legend) - 1)
        if self.legend_top:
            ly = self.y + 18
        if self.legend:
            width = max(len(l[2]) for l in self.legend) * 5.0 + 26
            out.append(f'<rect x="{_f(lx - 3)}" y="{_f(ly - 9)}" width="{_f(width)}" '
                       f'height="{_f(11 * len(self.legend) + 2)}" fill="#FFFFFF" fill-opacity="0.85" stroke="#DADDE2"/>')
        for i, (kind, color, label) in enumerate(self.legend):
            yy = ly + 11 * i
            if kind == "marker":
                out.append(f'<circle cx="{_f(lx + 7)}" cy="{_f(yy - 3)}" r="2.2" fill="{color}"/>')
            elif kind == "tick":
                out.append(f'<line x1="{_f(lx + 7)}" y1="{_f(yy - 7)}" x2="{_f(lx + 7)}" y2="{_f(yy + 1)}" '
                           f'stroke="{color}" stroke-width="2"/>')
            else:
                dash = ' stroke-dasharray="4 3"' if kind == "dash" else ""
                out.append(f'<line x1="{_f(lx)}" y1="{_f(yy - 3)}" x2="{_f(lx + 14)}" y2="{_f(yy - 3)}" '
                           f'stroke="{color}" stroke-width="2"{dash}/>')
            out.append(f'<text x="{_f(lx + 18)}" y="{_f(yy)}" font-size="9" fill="{PALETTE["text"]}">'
                       f'{html.escape(label)}</text>')
        out.append("</g>")
        return "".join(out)


class Figure:
    def __init__(self, width, height, title, desc, subtitle=""):
        self.width, self.height, self.title, self.desc, self.subtitle = width, height, title, desc, subtitle
        self.panels: list[Panel] = []
        self.extra: list[str] = []

    def panel(self, *args, **kwargs) -> Panel:
        p = Panel(self, *args, **kwargs)
        self.panels.append(p)
        return p

    def note(self, x, y, s, size=9.5, color=None, weight="normal"):
        self.extra.append(f'<text x="{_f(x)}" y="{_f(y)}" font-size="{size}" font-weight="{weight}" '
                          f'fill="{color or PALETTE["text"]}">{html.escape(s)}</text>')

    def render(self) -> str:
        head = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.width}" height="{self.height}" '
                f'viewBox="0 0 {self.width} {self.height}" font-family="Segoe UI, Helvetica, Arial, sans-serif" '
                f'role="img" aria-labelledby="t d"><title id="t">{html.escape(self.title)}</title>'
                f'<desc id="d">{html.escape(self.desc)}</desc>'
                f'<rect width="100%" height="100%" fill="#FFFFFF"/>'
                f'<text x="24" y="26" font-size="16" font-weight="bold" fill="{PALETTE["text"]}">{html.escape(self.title)}</text>'
                f'<text x="24" y="44" font-size="10.5" fill="{PALETTE["grey"]}">{html.escape(self.subtitle)}</text>')
        return head + "".join(p.render() for p in self.panels) + "".join(self.extra) + "</svg>\n"
