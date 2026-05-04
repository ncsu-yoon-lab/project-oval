"""
Wolfwagen GUI — Project OVAL
Animated voice orb, NC State branding, vehicle data cards, focus mode.

Queue message protocol:
    ("status", str)   — updates VEHICLE STATUS label + drives orb animation
    ("detail", str)   — updates ACTIVE COMMAND card
    ("output", str)   — updates TRANSMISSION (response) card + focus overlay
"""

from __future__ import annotations

import math
import os
import queue
import tkinter as tk
from tkinter import font as tkFont


class VoiceOrb:
    """Morphing multi-layer blob. Layers are painted back→front (outermost first)."""

    LAYER_COLORS = [
        "#1A0000",
        "#3D0000",
        "#770000",
        "#AA0000",
        "#CC0000",
        "#EE3311",
        "#FF9977",
        "#FFFFFF",
    ]
    # (radial_scale, amplitude_factor, phase_offset)
    LAYER_CFG = [
        (2.00, 0.28, 0.0),
        (1.70, 0.42, 0.9),
        (1.42, 0.60, 1.8),
        (1.16, 0.80, 2.7),
        (0.90, 1.00, 0.4),
        (0.64, 0.68, 1.3),
        (0.38, 0.38, 2.2),
        (0.18, 0.16, 0.8),
    ]

    def __init__(self, canvas: tk.Canvas, cx: int, cy: int, base_r: float = 88.0):
        self.canvas = canvas
        self.cx, self.cy = cx, cy
        self.base_r = base_r
        self.phase = 0.0
        self.amplitude = 5.0
        self.target_amp = 5.0
        self.speed = 0.022
        self.target_speed = 0.022
        self.pulse_phase = 0.0
        self.pulse_amp = 0.0
        self.target_pulse_amp = 0.0
        self.pulse_speed = 0.0
        self.target_pulse_speed = 0.0

        self.rings = []
        for r in range(200, 0, -40):
            ring = canvas.create_oval(
                cx - r, cy - r, cx + r, cy + r, outline="#1C1C1C", fill="", width=1
            )
            self.rings.append((ring, r))

        self.layers = [
            canvas.create_polygon([0, 0, 1, 1], smooth=True, fill=c, outline="")
            for c in self.LAYER_COLORS
        ]
        self._animate()

    def update_center(self, cx: int, cy: int):
        self.cx = cx
        self.cy = cy
        for ring, r in self.rings:
            self.canvas.coords(ring, cx - r, cy - r, cx + r, cy + r)

    def set_state(self, state: str):
        if state == "speaking":
            self.target_amp = 16.0
            self.target_speed = 0.06
            self.target_pulse_amp = 24.0
            self.target_pulse_speed = 0.20
        elif state == "listening":
            self.target_amp = 14.0
            self.target_speed = 0.040
            self.target_pulse_amp = 0.0
            self.target_pulse_speed = 0.0
        else:
            self.target_amp = 5.0
            self.target_speed = 0.020
            self.target_pulse_amp = 0.0
            self.target_pulse_speed = 0.0

    def _pts(self, r: float, amp: float, p_off: float, n: int = 80):
        pts = []
        for i in range(n):
            a = 2 * math.pi * i / n
            radius = r + amp * (
                math.sin(self.phase + p_off + a * 3) * 0.50
                + math.sin(self.phase * 1.3 + p_off + a * 5) * 0.30
                + math.sin(self.phase * 0.7 + p_off + a * 2) * 0.20
            )
            pts.extend([self.cx + radius * math.cos(a), self.cy + radius * math.sin(a)])
        return pts

    def _animate(self):
        self.amplitude += (self.target_amp - self.amplitude) * 0.08
        self.speed += (self.target_speed - self.speed) * 0.05
        self.pulse_amp += (self.target_pulse_amp - self.pulse_amp) * 0.07
        self.pulse_speed += (self.target_pulse_speed - self.pulse_speed) * 0.05
        self.phase += self.speed
        self.pulse_phase += self.pulse_speed

        beat = self.pulse_amp * abs(math.sin(self.pulse_phase))
        effective_r = self.base_r + beat

        for layer, (sf, af, po) in zip(self.layers, self.LAYER_CFG):
            self.canvas.coords(
                layer, self._pts(effective_r * sf, self.amplitude * af, po)
            )
        self.canvas.after(30, self._animate)


class WolfwagenGUI:
    C_BG     = "#0A0A0A"
    C_PANEL  = "#111111"
    C_SURFACE = "#161616"
    C_RED    = "#CC0000"
    C_WHITE  = "#F5F5F5"
    C_TEXT   = "#E8E8E8"
    C_DIM    = "#5A5A5A"
    C_BORDER = "#242424"

    def __init__(self, root: tk.Tk, q: queue.Queue):
        self.root = root
        self.queue = q
        self.root.title("Project OVAL — Wolfwagen")
        self.root.geometry("1440x810")
        self.root.configure(bg=self.C_BG)
        self.root.resizable(True, True)
        self._focus_mode = False

        self._build_header()
        self._build_body()
        self._add_toggle_button()
        self.process_queue()

    # ── Header ────────────────────────────────────────────────────────────────
    def _build_header(self):
        hdr = tk.Frame(self.root, bg=self.C_PANEL, height=58)
        hdr.pack(fill="x", side="top")
        hdr.pack_propagate(False)
        self._hdr_frame = hdr

        f_title = tkFont.Font(family="Courier", size=18, weight="bold")
        f_sub   = tkFont.Font(family="Courier", size=9)
        f_badge = tkFont.Font(family="Courier", size=14, weight="bold")

        left = tk.Frame(hdr, bg=self.C_PANEL)
        left.pack(side="left", padx=24, pady=10)
        tk.Label(left, text="PROJECT OVAL", font=f_title, fg=self.C_WHITE, bg=self.C_PANEL).pack(anchor="w")
        tk.Label(left, text="AUTONOMOUS PATROL VEHICLE  ·  YOON'S LAB",
                 font=f_sub, fg=self.C_DIM, bg=self.C_PANEL).pack(anchor="w")

        right = tk.Frame(hdr, bg=self.C_PANEL)
        right.pack(side="right", padx=24, pady=10)
        tk.Label(right, text="WOLFWAGEN", font=f_badge, fg=self.C_RED, bg=self.C_PANEL).pack(anchor="e")
        tk.Label(right, text="NC STATE UNIVERSITY  ·  CENTENNIAL CAMPUS",
                 font=f_sub, fg=self.C_DIM, bg=self.C_PANEL).pack(anchor="e")

        self._red_line = tk.Frame(self.root, bg=self.C_RED, height=2)
        self._red_line.pack(fill="x", side="top")

    # ── Body ──────────────────────────────────────────────────────────────────
    def _build_body(self):
        body = tk.Frame(self.root, bg=self.C_BG)
        body.pack(fill="both", expand=True)
        self._body = body

        left = tk.Frame(body, bg=self.C_PANEL, width=280)
        left.pack(side="left", fill="y")
        left.pack_propagate(False)
        self._left_col = left

        self._left_div = tk.Frame(body, bg=self.C_BORDER, width=1)
        self._left_div.pack(side="left", fill="y")

        center = tk.Frame(body, bg=self.C_BG, width=460)
        center.pack(side="left", fill="y")
        center.pack_propagate(False)
        self._center_col = center

        self._right_div = tk.Frame(body, bg=self.C_BORDER, width=1)
        self._right_div.pack(side="left", fill="y")

        right = tk.Frame(body, bg=self.C_BG)
        right.pack(side="left", fill="both", expand=True)
        self._right_col = right

        self._build_left_col(left)
        self._build_center_col(center)
        self._build_right_col(right)

    # ── Left column ───────────────────────────────────────────────────────────
    def _build_left_col(self, panel):
        self.logo_label = tk.Label(panel, bg=self.C_PANEL)
        self.logo_label.pack(pady=(18, 0))
        try:
            from PIL import Image, ImageTk
            _here = os.path.dirname(os.path.abspath(__file__))
            img = Image.open(os.path.join(_here, "nc_state_logo.png"))
            img = img.resize((244, 244), Image.LANCZOS)
            self.logo_image = ImageTk.PhotoImage(img)
            self.logo_label.config(image=self.logo_image)
        except Exception:
            f_fb = tkFont.Font(family="Courier", size=13, weight="bold")
            self.logo_label.config(text="WOLFPACK", font=f_fb, fg=self.C_RED)

        tk.Frame(panel, bg=self.C_RED, height=2).pack(fill="x", padx=16, pady=14)

        f_key = tkFont.Font(family="Courier", size=8)
        f_val = tkFont.Font(family="Courier", size=10, weight="bold")

        rows = [
            ("UNIT",   "WOLFWAGEN-01"),
            ("CAMPUS", "CENTENNIAL"),
            ("MODE",   "PATROL"),
            ("LLM",    "LLAMA 3.2"),
        ]
        data = tk.Frame(panel, bg=self.C_PANEL)
        data.pack(fill="x", padx=16)
        for key, val in rows:
            row = tk.Frame(data, bg=self.C_PANEL)
            row.pack(fill="x", pady=4)
            tk.Label(row, text=key, font=f_key, fg=self.C_DIM, bg=self.C_PANEL,
                     width=8, anchor="w").pack(side="left")
            tk.Frame(row, bg=self.C_BORDER, width=1).pack(side="left", fill="y", padx=6)
            tk.Label(row, text=val, font=f_val, fg=self.C_WHITE, bg=self.C_PANEL,
                     anchor="w").pack(side="left")

        tk.Frame(panel, bg=self.C_BORDER, height=1).pack(fill="x", padx=16, pady=(18, 0))

        bottom = tk.Frame(panel, bg=self.C_PANEL)
        bottom.pack(fill="both", expand=True, padx=16, pady=(14, 20))

        f_desc_lbl = tkFont.Font(family="Courier", size=9, weight="bold")
        f_desc     = tkFont.Font(family="Courier", size=11)

        tk.Label(bottom, text="ABOUT", font=f_desc_lbl, fg=self.C_DIM, bg=self.C_PANEL).pack(anchor="w", pady=(0, 10))
        tk.Label(
            bottom,
            text=(
                "Wolfwagen is an autonomous\n"
                "patrol vehicle developed by\n"
                "Yoon's Lab at NC State\n"
                "University.\n\n"
                "It navigates Centennial\n"
                "Campus using computer\n"
                "vision, GPS, and a local\n"
                "LLM voice interface."
            ),
            font=f_desc, fg="#707070", bg=self.C_PANEL, justify="left", anchor="nw",
        ).pack(anchor="nw")

    # ── Center column: voice orb ──────────────────────────────────────────────
    def _build_center_col(self, panel):
        orb_canvas = tk.Canvas(panel, bg=self.C_BG, highlightthickness=0, width=460, height=720)
        orb_canvas.pack(fill="both", expand=True)
        self.voice_orb = VoiceOrb(orb_canvas, cx=230, cy=360, base_r=88)

        def on_resize(event):
            self.voice_orb.update_center(event.width // 2, event.height // 2)

        orb_canvas.bind("<Configure>", on_resize)

    # ── Right column ──────────────────────────────────────────────────────────
    def _build_right_col(self, panel):
        f_lbl = tkFont.Font(family="Courier", size=9, weight="bold")

        def card(parent, top_accent=None, fixed_h=None):
            outer = tk.Frame(parent, bg=self.C_SURFACE)
            if fixed_h:
                outer.config(height=fixed_h)
                outer.pack_propagate(False)
            if top_accent:
                tk.Frame(outer, bg=top_accent, height=2).pack(fill="x")
            inner = tk.Frame(outer, bg=self.C_SURFACE)
            inner.pack(fill="both", expand=True, padx=22, pady=14)
            return outer, inner

        # VEHICLE STATUS
        sc, si = card(panel, top_accent=self.C_RED, fixed_h=104)
        sc.pack(fill="x", padx=22, pady=(20, 8))
        tk.Label(si, text="VEHICLE STATUS", font=f_lbl, fg=self.C_DIM, bg=self.C_SURFACE).pack(anchor="w")

        dot_row = tk.Frame(si, bg=self.C_SURFACE)
        dot_row.pack(anchor="w", pady=(5, 0))
        self._dot_cv = tk.Canvas(dot_row, width=14, height=14, bg=self.C_SURFACE, highlightthickness=0)
        self._dot_cv.pack(side="left", padx=(0, 10))
        self._dot = self._dot_cv.create_oval(1, 1, 13, 13, fill=self.C_RED, outline="")

        f_status = tkFont.Font(family="Helvetica", size=26, weight="bold")
        self.status_label = tk.Label(dot_row, text="Initializing...", font=f_status,
                                     fg=self.C_TEXT, bg=self.C_SURFACE)
        self.status_label.pack(side="left")

        # ACTIVE COMMAND
        ac, ai = card(panel, fixed_h=76)
        ac.pack(fill="x", padx=22, pady=(0, 8))
        tk.Label(ai, text="ACTIVE COMMAND", font=f_lbl, fg=self.C_DIM, bg=self.C_SURFACE).pack(anchor="w")

        f_detail = tkFont.Font(family="Courier", size=13)
        self.detail_label = tk.Label(ai, text="—", font=f_detail, fg="#888888",
                                     bg=self.C_SURFACE, wraplength=640, justify="left")
        self.detail_label.pack(anchor="w", pady=(3, 0))

        # TRANSMISSION
        tc, ti = card(panel, top_accent=self.C_RED)
        tc.pack(fill="both", expand=True, padx=22, pady=(0, 22))
        tk.Label(ti, text="TRANSMISSION", font=f_lbl, fg=self.C_DIM, bg=self.C_SURFACE).pack(anchor="nw")

        f_response = tkFont.Font(family="Helvetica", size=21, weight="bold")
        self.output_label = tk.Label(ti, text="", font=f_response, fg=self.C_TEXT,
                                     bg=self.C_SURFACE, wraplength=640,
                                     justify="center", anchor="center")
        self.output_label.pack(fill="both", expand=True, pady=(7, 0))

    # ── Focus toggle ──────────────────────────────────────────────────────────
    def _add_toggle_button(self):
        f_btn = tkFont.Font(family="Courier", size=9, weight="bold")
        self._toggle_btn = tk.Button(
            self.root, text="[ FOCUS ]", font=f_btn, fg=self.C_DIM,
            bg=self.C_PANEL, activebackground=self.C_PANEL, activeforeground=self.C_RED,
            bd=0, highlightthickness=1, highlightbackground=self.C_BORDER,
            relief="flat", padx=10, pady=4, cursor="hand2", command=self.toggle_focus,
        )
        self._toggle_btn.place(relx=0.99, rely=0.01, anchor="ne")

        f_focus_title = tkFont.Font(family="Courier", size=36, weight="bold")
        self._focus_title = tk.Label(self.root, text="WOLFWAGEN", font=f_focus_title,
                                     fg=self.C_RED, bg=self.C_BG)

        f_focus_output = tkFont.Font(family="Helvetica", size=26, weight="bold")
        self._focus_output = tk.Label(self.root, text="", font=f_focus_output,
                                      fg=self.C_TEXT, bg=self.C_BG,
                                      wraplength=1100, justify="center")

    def toggle_focus(self):
        self._focus_mode = not self._focus_mode
        if self._focus_mode:
            self._hdr_frame.pack_forget()
            self._red_line.pack_forget()
            self._left_col.pack_forget()
            self._left_div.pack_forget()
            self._right_div.pack_forget()
            self._right_col.pack_forget()
            self._center_col.pack_forget()
            self._center_col.pack(fill="both", expand=True)
            self._focus_title.place(relx=0.5, rely=0.06, anchor="n")
            self._focus_output.place(relx=0.5, rely=0.92, anchor="s")
            self._toggle_btn.config(text="[ EXIT ]", fg=self.C_RED)
        else:
            self._focus_title.place_forget()
            self._focus_output.place_forget()
            self._center_col.pack_forget()
            self._hdr_frame.pack(fill="x", side="top", before=self._body)
            self._red_line.pack(fill="x", side="top", after=self._hdr_frame)
            self._left_col.pack(side="left", fill="y")
            self._left_div.pack(side="left", fill="y")
            self._center_col.pack(side="left", fill="y")
            self._right_div.pack(side="left", fill="y")
            self._right_col.pack(side="left", fill="both", expand=True)
            self._toggle_btn.config(text="[ FOCUS ]", fg=self.C_DIM)

    # ── Queue ─────────────────────────────────────────────────────────────────
    def process_queue(self):
        try:
            while not self.queue.empty():
                msg_type, value = self.queue.get_nowait()
                if msg_type == "status":
                    self.status_label.config(text=value)
                    t = value.lower()
                    if "speak" in t:
                        self.voice_orb.set_state("speaking")
                    elif any(w in t for w in ("listen", "ready", "waiting", "calibrat")):
                        self.voice_orb.set_state("listening")
                    else:
                        self.voice_orb.set_state("idle")
                elif msg_type == "detail":
                    self.detail_label.config(text=value if value else "—")
                elif msg_type == "output":
                    self.output_label.config(text=value)
                    self._focus_output.config(text=value)
        finally:
            self.root.after(50, self.process_queue)
