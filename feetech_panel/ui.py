"""Russian-language desktop interface for the Feetech STS bus."""

import tkinter as tk
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from math import ceil, floor, log10
from queue import Empty, Queue
from time import monotonic
from tkinter import font, messagebox, simpledialog, ttk

from .bus import BAUD_RATES, MAX_ID, MAX_POSITION, MODELS, SPEED_MODE, BusError, Config, Feedback, Identity, Motion, status_text
from .worker import BusWorker


BACKGROUND = "#edf2f8"
PAPER = "#ffffff"
INK = "#15283c"
MUTED = "#4e6378"
LINE = "#d5dee9"
HEADER = "#132c46"
BLUE = "#1758af"
AMBER = "#a95519"
GREEN = "#13744e"
ERROR = "#a12d39"
HISTORY_SECONDS = 30
READING_VALUE_SIZE = 23
READING_CARD_GAP = 5
READING_CARD_PADDING = 4
SUMMARY_COLUMNS = ("id", "model", "sent", "goal", "position", "delta", "torque", "status")


@dataclass
class Row:
    identity: Identity
    config: Config | None
    feedback: Feedback | None = None
    motion: Motion | None = None
    issue: str = ""
    history: deque = field(default_factory=lambda: deque(maxlen=180))


def graph_bounds(values):
    bottom, top = min(values), max(values)
    span = max(40, top - bottom)
    base = 10 ** floor(log10(span / 4))
    step = next(base * unit for unit in (1, 2, 5, 10) if base * unit >= span / 4)
    middle = (top + bottom) / 2
    return floor((middle - span / 2) / step) * step, ceil((middle + span / 2) / step) * step, step


def spaced_ticks(ticks, span, gap):
    """Leave room between labels and keep both endpoints when they fit."""
    if span < gap:
        return (ticks[-1],)
    last = len(ticks) - 1
    stride = max(1, ceil(last * gap / span))
    return (ticks[0], *ticks[stride:last - stride + 1:stride], ticks[-1])


class PositionGraph(tk.Canvas):
    def __init__(self, parent):
        super().__init__(parent, background=PAPER, highlightthickness=0, width=300, height=118)
        self.history = ()
        self.empty_text = "Выберите один мотор в таблице"
        self.axis_font = font.nametofont("TkDefaultFont", root=self).copy()
        self.axis_font.configure(size=10)
        self.bind("<Configure>", lambda _event: self.draw())

    def draw(self):
        self.delete("all")
        width, height = self.winfo_width(), self.winfo_height()
        if width < 40 or height < 35:
            return
        now = monotonic()
        visible = [sample for sample in self.history if sample[0] >= now - HISTORY_SECONDS]
        values = [value for _, *positions in visible for value in positions if value is not None]
        if not values:
            self.create_text(width / 2, height / 2, text=self.empty_text, fill=MUTED, width=width - 20)
            return

        minimum, maximum, step = graph_bounds(values)
        position_ticks = range(minimum, maximum + 1, step)
        time_labels = {seconds: f"−{seconds} с" if seconds else "0 с" for seconds in (30, 20, 10, 0)}
        label_height = self.axis_font.metrics("linespace")
        time_gap = max(map(self.axis_font.measure, time_labels.values())) + 8
        position_gap = label_height + 4
        left = max(max(self.axis_font.measure(str(tick)) for tick in position_ticks) + 11,
                   self.axis_font.measure(time_labels[30]) // 2 + 4)
        right = width - self.axis_font.measure(time_labels[0]) // 2 - 4
        label_half_height = (label_height + 1) // 2
        time_label_offset = label_half_height + 4
        top = label_half_height + 2
        bottom = height - label_height - time_label_offset - 3
        if right <= left or bottom <= top:
            return

        def x(timestamp):
            return right - (now - timestamp) * (right - left) / HISTORY_SECONDS

        def y(position):
            return bottom - (position - minimum) * (bottom - top) / (maximum - minimum)

        for position in spaced_ticks(position_ticks, bottom - top, position_gap):
            mark = y(position)
            self.create_line(left, mark, right, mark, fill=LINE)
            self.create_text(left - 8, mark, anchor="e", text=str(position), font=self.axis_font, fill=MUTED)
        for seconds in spaced_ticks(tuple(time_labels), right - left, time_gap):
            mark = x(now - seconds)
            self.create_line(mark, top, mark, bottom, fill=LINE)
            self.create_text(mark, bottom + time_label_offset, anchor="n", text=time_labels[seconds],
                             font=self.axis_font, fill=MUTED)

        for index, color, dash in ((1, AMBER, (6, 3)), (2, BLUE, ())):
            series = []
            for point in visible:
                if point[index] is None:
                    self._draw_series(series, color, dash)
                    series.clear()
                else:
                    series.extend((x(point[0]), y(point[index])))
            self._draw_series(series, color, dash)

    def _draw_series(self, coordinates, color, dash):
        if len(coordinates) == 2:
            x, y = coordinates
            self.create_oval(x - 2, y - 2, x + 2, y + 2, fill=color, outline="")
        elif coordinates:
            self.create_line(*coordinates, fill=color, width=2, dash=dash)


class Panel:
    def __init__(self, root, demo=False):
        self.root = root
        self.events = Queue()
        self.rows = {}
        self.worker = None
        self.connected = False
        self.busy = None
        self.closing = False
        self.port_failed = False
        self.selection = ()
        self.range = None
        self.sweep_id = None
        self.port = tk.StringVar()
        self.baud_rate = tk.StringVar(value=str(BAUD_RATES[0]))
        self.first_id = tk.StringVar(value="0")
        self.last_id = tk.StringVar(value=str(MAX_ID))
        self.target = tk.StringVar()
        self.slider_value = tk.DoubleVar()
        self.sweep_amplitude = tk.DoubleVar(value=50)
        self.speed = tk.StringVar(value="20")
        self.acceleration = tk.StringVar(value="10")
        self.full_table = tk.BooleanVar()
        self.logs = deque(maxlen=250)
        self.log_window = None
        self._configure_window()
        self._build()
        self.refresh_ports()
        self._refresh_controls()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(35, self._consume_events)
        root.after(300, self._refresh_times)
        if demo:
            root.after_idle(lambda: self.connect(demo=True))

    def _configure_window(self):
        self.root.title("Feetech STS · Панель сервоприводов")
        self.root.configure(background=BACKGROUND)
        self.root.minsize(860, 640)
        width = min(1360, self.root.winfo_screenwidth() - 48)
        height = min(875, self.root.winfo_screenheight() - 78)
        self.root.geometry(f"{max(860, width)}x{max(640, height)}")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(2, weight=1)
        default = font.nametofont("TkDefaultFont")
        default.configure(size=13)
        for name in ("TkTextFont", "TkMenuFont", "TkHeadingFont", "TkCaptionFont"):
            font.nametofont(name).configure(size=13)
        family = default.actual("family")
        self.position_font = font.Font(root=self.root, family=family, size=18, weight="bold")
        self.reading_font = font.Font(root=self.root, family=family, size=READING_VALUE_SIZE, weight="bold")
        self.reading_font_probe = self.reading_font.copy()
        self.reading_value_width = 0
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure(".", foreground=INK, font=default)
        style.configure("TFrame", background=BACKGROUND)
        style.configure("Paper.TFrame", background=PAPER)
        style.configure("Header.TFrame", background=HEADER)
        style.configure("Reading.TFrame", background=BACKGROUND)
        style.configure("TLabel", background=PAPER, foreground=INK)
        style.configure("Title.TLabel", background=HEADER, foreground=PAPER, font=(family, 24, "bold"))
        style.configure("Subtitle.TLabel", background=HEADER, foreground="#cfdeef")
        style.configure("Section.TLabel", font=(family, 16, "bold"))
        style.configure("Note.TLabel", foreground=MUTED)
        style.configure("Blue.TLabel", foreground=BLUE)
        style.configure("Amber.TLabel", foreground=AMBER)
        style.configure("ReadingCaption.TLabel", background=BACKGROUND, foreground=MUTED, font=(family, 12))
        style.configure("ReadingValue.TLabel", background=BACKGROUND, font=self.reading_font)
        style.configure("Goal.ReadingValue.TLabel", foreground=AMBER)
        style.configure("Actual.ReadingValue.TLabel", foreground=BLUE)
        style.configure("TButton", padding=(10, 7), background=BACKGROUND, bordercolor=LINE)
        style.map("TButton", background=[("disabled", "#f3f5f8"), ("active", "#dbe6f4")])
        style.configure("Primary.TButton", foreground=PAPER, background=BLUE, bordercolor=BLUE, font=(family, 13, "bold"))
        style.map("Primary.TButton", foreground=[("disabled", MUTED)], background=[("disabled", BACKGROUND), ("active", "#10458c")])
        style.configure("Release.TButton", foreground=ERROR, background="#fbeef0", bordercolor="#eac7cd")
        style.map("Release.TButton", background=[("disabled", "#f3f5f8"), ("active", "#f6d8de")])
        style.configure("TCheckbutton", background=PAPER)
        style.configure("TCombobox", padding=(5, 4))
        style.configure("TSpinbox", padding=(5, 4), arrowsize=14)
        style.configure("Treeview", background=PAPER, fieldbackground=PAPER,
                        rowheight=max(39, default.metrics("linespace") + 14), borderwidth=0)
        style.map("Treeview", background=[("selected", "#d8e7fc")], foreground=[("selected", INK)])
        style.configure("Treeview.Heading", background="#e5edf7", foreground=INK, font=(family, 12, "bold"), padding=(6, 8))
        style.configure("TProgressbar", background=BLUE, troughcolor=LINE)
        style.configure("Horizontal.TScale", background="#b5c9e4", troughcolor="#e3ebf5", sliderlength=30)

    def _build(self):
        top = ttk.Frame(self.root, style="Header.TFrame", padding=(18, 10, 18, 11))
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(0, weight=1)
        ttk.Label(top, text="Панель сервоприводов", style="Title.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(top, text="Feetech ST3215 / ST3250  ·  STS TTL  ·  12 В", style="Subtitle.TLabel").grid(row=1, column=0, sticky="w")
        self.connection = tk.Label(top, padx=12, pady=6, text="Отключено", bg="#dce5f1", fg=MUTED)
        self.connection.grid(row=0, column=1, rowspan=2, sticky="e", padx=(10, 0))

        toolbar = ttk.Frame(self.root, style="Paper.TFrame", padding=(10, 7))
        toolbar.grid(row=1, column=0, sticky="ew", padx=14, pady=8)
        toolbar.columnconfigure(1, weight=1)
        ttk.Label(toolbar, text="Порт").grid(row=0, column=0, padx=(0, 7))
        self.port_box = ttk.Combobox(toolbar, textvariable=self.port, width=13)
        self.port_box.grid(row=0, column=1, sticky="ew", padx=(0, 8))
        self.ports_button = ttk.Button(toolbar, text="Обновить", command=self.refresh_ports)
        self.ports_button.grid(row=0, column=2, padx=(0, 12))
        ttk.Label(toolbar, text="Бод").grid(row=0, column=3, padx=(0, 6))
        self.baud_box = ttk.Combobox(toolbar, textvariable=self.baud_rate, values=BAUD_RATES, state="readonly", width=8)
        self.baud_box.grid(row=0, column=4, padx=(0, 8))
        self.connect_button = ttk.Button(toolbar, text="Подключить", width=13, style="Primary.TButton", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=5, padx=(0, 8))
        self.demo_button = ttk.Button(toolbar, text="Демо", command=lambda: self.connect(demo=True))
        self.demo_button.grid(row=0, column=6, padx=(0, 8))
        ttk.Button(toolbar, text="Журнал", command=self.open_log).grid(row=0, column=7)

        body = ttk.Frame(self.root)
        body.grid(row=2, column=0, sticky="nsew", padx=14)
        body.columnconfigure(0, weight=1)
        body.rowconfigure(0, weight=3, minsize=154)
        table_panel = ttk.Frame(body, style="Paper.TFrame", padding=(10, 7, 10, 8))
        table_panel.grid(row=0, column=0, sticky="nsew", pady=(0, 8))
        self._build_table(table_panel)

        details = ttk.Frame(body)
        details.grid(row=1, column=0, sticky="nsew")
        details.columnconfigure(1, weight=1)
        details.rowconfigure(0, weight=1)
        controls = ttk.Frame(details, style="Paper.TFrame", padding=12)
        controls.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        self._build_controls(controls)
        graph_panel = ttk.Frame(details, style="Paper.TFrame", padding=12)
        graph_panel.grid(row=0, column=1, sticky="nsew")
        self._build_graph(graph_panel)

        footer = ttk.Frame(self.root, padding=(15, 5, 15, 6))
        footer.grid(row=3, column=0, sticky="ew")
        footer.columnconfigure(0, weight=1)
        self.status = tk.Label(footer, text="Выберите последовательный порт или запустите демо.", bg=BACKGROUND, fg=MUTED, anchor="w", width=1)
        self.status.grid(row=0, column=0, sticky="ew")
        self.progress = ttk.Progressbar(footer, mode="determinate", length=125)
        self.progress.grid(row=0, column=1, padx=(10, 0))
        self.progress.grid_remove()

    def _build_table(self, parent):
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(1, weight=1)
        header = ttk.Frame(parent, style="Paper.TFrame")
        header.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 7))
        header.columnconfigure(1, weight=1)
        self.table_title = ttk.Label(header, text="Моторы", style="Section.TLabel")
        self.table_title.grid(row=0, column=0, padx=(0, 12))
        self.select_online = ttk.Button(header, text="Выбрать доступные", command=self.select_available)
        self.select_online.grid(row=0, column=1, sticky="w")
        ttk.Label(header, text="ID от").grid(row=0, column=2, padx=(8, 6))
        self.first_box = ttk.Spinbox(header, from_=0, to=MAX_ID, textvariable=self.first_id, width=3)
        self.first_box.grid(row=0, column=3)
        ttk.Label(header, text="до").grid(row=0, column=4, padx=5)
        self.last_box = ttk.Spinbox(header, from_=0, to=MAX_ID, textvariable=self.last_id, width=3)
        self.last_box.grid(row=0, column=5)
        self.scan_button = ttk.Button(header, text="Найти моторы", width=16, command=self.scan)
        self.scan_button.grid(row=0, column=6, padx=(8, 0))

        columns = (
            ("id", "ID", 50), ("model", "Модель", 94), ("sent", "Подано", 87),
            ("goal", "Цель", 72), ("position", "Факт", 72), ("delta", "Разница", 92),
            ("speed", "Скорость", 98), ("voltage", "В", 56), ("temp", "°C", 52), ("current", "А", 56),
            ("torque", "Удержание", 111), ("status", "Состояние", 138),
        )
        column_keys = tuple(key for key, _, _ in columns)
        self.full_columns = SUMMARY_COLUMNS + tuple(key for key in column_keys if key not in SUMMARY_COLUMNS)
        self.table = ttk.Treeview(parent, columns=column_keys, displaycolumns=SUMMARY_COLUMNS, show="headings", height=4, selectmode="extended")
        for key, label, width in columns:
            self.table.heading(key, text=label)
            self.table.column(key, width=width, minwidth=width, stretch=key != "id", anchor="w" if key in ("model", "status") else "center")
        self.table.tag_configure("stripe", background="#f3f7fb")
        self.table.tag_configure("warning", foreground=ERROR)
        self.table.tag_configure("stale", foreground=MUTED)
        self.table.grid(row=1, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(parent, orient="vertical", command=self.table.yview)
        scroll.grid(row=1, column=1, sticky="ns")
        self.table_scroll = ttk.Scrollbar(parent, orient="horizontal", command=self.table.xview)
        self.table_scroll.grid(row=2, column=0, sticky="ew")
        self.table_scroll.grid_remove()
        self.table.configure(yscrollcommand=scroll.set, xscrollcommand=self._table_xscroll)
        self.table.bind("<<TreeviewSelect>>", self._select)

        options = ttk.Frame(parent, style="Paper.TFrame")
        options.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(5, 0))
        options.columnconfigure(0, weight=1)
        self.selection_label = ttk.Label(options, text="Ctrl / Shift для выбора группы", style="Note.TLabel")
        self.selection_label.grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(options, text="Все столбцы", variable=self.full_table, command=self.toggle_columns).grid(row=0, column=1)

    def _build_controls(self, parent):
        parent.columnconfigure(0, weight=1)
        header = ttk.Frame(parent, style="Paper.TFrame")
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(0, weight=1)
        self.control_title = ttk.Label(header, text="Управление", style="Section.TLabel")
        self.control_title.grid(row=0, column=0, sticky="w", padx=(0, 8))
        self.id_button = ttk.Button(header, text="Сменить ID", command=self.change_id)
        self.id_button.grid(row=0, column=1, sticky="e")

        self.control_note = tk.Label(parent, text="Выберите мотор в таблице.", bg=PAPER, fg=MUTED,
                                     height=2, width=1, wraplength=345, justify="left", anchor="nw")
        self.control_note.grid(row=1, column=0, sticky="ew", pady=(1, 5))

        position = ttk.Frame(parent, style="Paper.TFrame")
        position.grid(row=2, column=0, sticky="ew")
        position.columnconfigure(1, weight=1)
        ttk.Label(position, text="Позиция, отсч.").grid(row=0, column=0, sticky="w")
        self.position_box = ttk.Spinbox(position, from_=0, to=MAX_POSITION, width=6, font=self.position_font,
                                       justify="center", textvariable=self.target)
        self.position_box.grid(row=1, column=0, sticky="w")
        self.use_actual = ttk.Button(position, text="Взять факт", command=self.use_actual_position)
        self.use_actual.grid(row=1, column=1, sticky="e")
        self.position_mode_button = ttk.Button(position, text="Режим позиции", command=self.set_position_mode)
        self.position_mode_button.grid(row=1, column=1, sticky="e")
        self.position_mode_button.grid_remove()
        self.target.trace_add("write", self._target_edited)

        self.slider = ttk.Scale(parent, from_=0, to=MAX_POSITION, variable=self.slider_value, command=self._slide)
        self.slider.grid(row=3, column=0, sticky="ew", pady=(6, 0))
        limits = ttk.Frame(parent, style="Paper.TFrame")
        limits.grid(row=4, column=0, sticky="ew")
        limits.columnconfigure(0, weight=1)
        self.minimum_label = ttk.Label(limits, text="0", style="Note.TLabel")
        self.minimum_label.grid(row=0, column=0, sticky="w")
        self.maximum_label = ttk.Label(limits, text=str(MAX_POSITION), style="Note.TLabel")
        self.maximum_label.grid(row=0, column=1, sticky="e")

        values = ttk.Frame(parent, style="Paper.TFrame")
        values.grid(row=5, column=0, sticky="ew", pady=(5, 8))
        values.columnconfigure((0, 1), weight=1, uniform="motion_values")
        ttk.Label(values, text="Скорость").grid(row=0, column=0, sticky="w")
        ttk.Label(values, text="Ускорение").grid(row=0, column=1, sticky="w", padx=(8, 0))
        self.speed_box = ttk.Spinbox(values, from_=1, to=32767, width=6, textvariable=self.speed)
        self.speed_box.grid(row=1, column=0, sticky="ew", pady=(2, 0))
        self.acc_box = ttk.Spinbox(values, from_=1, to=254, width=5, textvariable=self.acceleration)
        self.acc_box.grid(row=1, column=1, sticky="ew", padx=(8, 0), pady=(2, 0))

        self.send_button = ttk.Button(parent, text="Отправить позицию", style="Primary.TButton", command=self.send)
        self.send_button.grid(row=6, column=0, sticky="ew", pady=(0, 6))
        for entry in (self.position_box, self.speed_box, self.acc_box):
            entry.bind("<Return>", self._send_on_enter)
            entry.bind("<KP_Enter>", self._send_on_enter)

        buttons = ttk.Frame(parent, style="Paper.TFrame")
        buttons.grid(row=7, column=0, sticky="ew")
        buttons.columnconfigure((0, 1), weight=1, uniform="torque")
        self.torque_on = ttk.Button(buttons, text="Удерживать", command=lambda: self._set_torque(True))
        self.torque_on.grid(row=0, column=0, sticky="ew", padx=(0, 4))
        self.torque_off = ttk.Button(buttons, text="Снять удержание", style="Release.TButton",
                                     command=lambda: self._set_torque(False))
        self.torque_off.grid(row=0, column=1, sticky="ew", padx=(4, 0))

        sweep = ttk.Frame(parent, style="Paper.TFrame")
        sweep.grid(row=8, column=0, sticky="ew", pady=(8, 0))
        sweep.columnconfigure(0, weight=1)
        self.sweep_value = ttk.Label(sweep, style="Note.TLabel")
        self.sweep_value.grid(row=0, column=0, sticky="w")
        self.sweep_slider = ttk.Scale(sweep, from_=1, to=MAX_POSITION, variable=self.sweep_amplitude)
        self.sweep_slider.grid(row=1, column=0, sticky="ew")
        self.sweep_button = ttk.Button(sweep, text="Вперёд-назад", command=self.toggle_sweep)
        self.sweep_button.grid(row=0, column=1, rowspan=2, padx=(8, 0), sticky="e")
        self.sweep_amplitude.trace_add("write", self._update_sweep_value)
        self._update_sweep_value()

    def _build_graph(self, parent):
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(3, weight=1)
        self.graph_title = ttk.Label(parent, text="Обратная связь", style="Section.TLabel")
        self.graph_title.grid(row=0, column=0, sticky="w")
        readouts = ttk.Frame(parent, style="Paper.TFrame")
        readouts.grid(row=1, column=0, sticky="ew", pady=(8, 7))
        self.readings = []
        for column, (title, style) in enumerate((
            ("Подано", "ReadingValue.TLabel"),
            ("Цель", "Goal.ReadingValue.TLabel"),
            ("Факт", "Actual.ReadingValue.TLabel"),
            ("Разница", "ReadingValue.TLabel"),
        )):
            readouts.columnconfigure(column, weight=1, uniform="readings")
            card = ttk.Frame(readouts, style="Reading.TFrame", padding=READING_CARD_PADDING)
            card.grid(row=0, column=column, sticky="ew", padx=(0, READING_CARD_GAP) if column < 3 else 0)
            card.columnconfigure(0, weight=1)
            ttk.Label(card, text=title, style="ReadingCaption.TLabel").grid(row=0, column=0)
            value = ttk.Label(card, text="—", style=style, width=1, anchor="center")
            value.grid(row=1, column=0, sticky="ew")
            self.readings.append(value)
        readouts.bind("<Configure>", self._fit_reading_font)

        legend = ttk.Frame(parent, style="Paper.TFrame")
        legend.grid(row=2, column=0, sticky="ew")
        legend.columnconfigure(3, weight=1)
        ttk.Label(legend, text="┄ Цель", style="Amber.TLabel").grid(row=0, column=0, padx=(0, 12))
        ttk.Label(legend, text="━ Факт", style="Blue.TLabel").grid(row=0, column=1, padx=(0, 12))
        ttk.Label(legend, text="Отсчёты · 30 с", style="Note.TLabel").grid(row=0, column=3, sticky="e")
        self.graph = PositionGraph(parent)
        self.graph.grid(row=3, column=0, sticky="nsew")
        self.measurements = tk.Label(parent, text="", bg=PAPER, fg=MUTED, height=3, width=1, justify="left", anchor="nw")
        self.measurements.grid(row=4, column=0, sticky="ew", pady=(3, 0))
        self.measurements.bind("<Configure>", self._wrap_measurements)

    def _fit_reading_font(self, event):
        count = len(self.readings)
        card_width = event.width // count - READING_CARD_GAP
        value_width = card_width - 2 * READING_CARD_PADDING - 2
        if value_width <= 0 or value_width == self.reading_value_width:
            return
        self.reading_value_width = value_width
        # The difference between two signed readings can have five digits and a sign.
        for size in range(READING_VALUE_SIZE, 1, -1):
            self.reading_font_probe.configure(size=size)
            digit = max(map(self.reading_font_probe.measure, "0123456789"))
            sign = max(map(self.reading_font_probe.measure, "+-"))
            if 5 * digit + sign <= value_width:
                break
        else:
            size = 1
        if self.reading_font.cget("size") != size:
            self.reading_font.configure(size=size)

    def _wrap_measurements(self, event):
        if int(self.measurements.cget("wraplength")) != event.width:
            self.measurements.configure(wraplength=event.width)

    def _table_xscroll(self, first, last):
        self.table_scroll.set(first, last)
        if float(first) <= 0 and float(last) >= 1:
            self.table_scroll.grid_remove()
        else:
            self.table_scroll.grid()

    def toggle_columns(self):
        self.table.configure(displaycolumns=self.full_columns if self.full_table.get() else SUMMARY_COLUMNS)
        self.table.xview_moveto(0)

    def refresh_ports(self):
        try:
            from serial.tools import list_ports
            ports = [port.device for port in sorted(list_ports.comports())]
        except (ImportError, OSError):
            ports = []
        self.port_box.configure(values=ports)
        if not self.port.get() and ports:
            self.port.set(ports[0])

    def connect(self, demo=False):
        if self.worker is not None:
            return
        device = self.port.get().strip()
        if not device and not demo:
            self.message("Выберите или введите путь к последовательному порту.", True)
            return
        self.table.delete(*self.table.get_children())
        self.rows.clear()
        self._select()
        self.busy = "connect"
        self.port_failed = False
        self.worker = BusWorker(device, int(self.baud_rate.get()), self.events, demo=demo)
        self.worker.start()
        self.message("Открываю шину…")
        self._refresh_controls()

    def toggle_connection(self):
        if self.worker is None:
            self.connect()
        else:
            self.worker.stop_requested.set()
            self.busy = "disconnect"
            self.message("Отключаюсь…")
            self._refresh_controls()

    def close(self):
        if self.worker is None:
            self.root.destroy()
        else:
            self.closing = True
            self.worker.stop_requested.set()
            self.busy = "disconnect"
            self._refresh_controls()

    def scan(self):
        if self.busy == "scan":
            self.worker.cancel_scan.set()
            self.scan_button.state(["disabled"])
            return
        try:
            first, last = int(self.first_id.get()), int(self.last_id.get())
            if not 0 <= first <= last <= MAX_ID:
                raise ValueError
        except ValueError:
            self.message(f"Укажите диапазон ID от 0 до {MAX_ID}.", True)
            return
        if self.dispatch("scan", first, last):
            self.progress.configure(value=0, maximum=last - first + 1)
            self.progress.grid()
            self.message("Ищу моторы…")

    def dispatch(self, operation, *args):
        if not self.connected or self.busy:
            return False
        self.busy = operation
        self.worker.submit(operation, *args)
        self._refresh_controls()
        return True

    def _set_torque(self, enabled):
        ids = self.selection
        if self.sweep_id is not None and not enabled:
            ids = (self.sweep_id,)
        if self.dispatch("torque", ids, enabled):
            self.message("Включаю удержание…" if enabled else "Снимаю удержание…")

    def _update_sweep_value(self, *_):
        self.sweep_value.configure(text=f"Амплитуда ±{round(self.sweep_amplitude.get())} отсч.")

    def toggle_sweep(self):
        if self.sweep_id is not None:
            if self.dispatch("stop_sweep"):
                self.message("Останавливаю движение вперёд-назад…")
            return
        if len(self.selection) != 1:
            return
        try:
            speed, acceleration = int(self.speed.get()), int(self.acceleration.get())
        except ValueError:
            self.message("Введите целые числа для скорости и ускорения.", True)
            return
        self.dispatch("start_sweep", self.selection[0], round(self.sweep_amplitude.get()), speed, acceleration)

    def _send_on_enter(self, _event):
        if self.send_button.instate(["!disabled"]):
            self.send_button.invoke()
        return "break"

    def send(self):
        try:
            motion = Motion(int(self.target.get()), int(self.speed.get()), int(self.acceleration.get()))
            if self.range and not self.range[0] <= motion.position <= self.range[1]:
                raise BusError(f"Для выбранных моторов доступен диапазон {self.range[0]}–{self.range[1]} отсч.")
        except ValueError:
            self.message("Введите целые числа для позиции, скорости и ускорения.", True)
            return
        except BusError as exc:
            self.message(str(exc), True)
            return
        self.dispatch("move", self.selection, motion)

    def use_actual_position(self):
        row = self.rows[self.selection[0]]
        self.target.set(str(row.feedback.position))

    def set_position_mode(self):
        if len(self.selection) != 1 or self.busy:
            return
        servo_id = self.selection[0]
        confirmed = messagebox.askyesno(
            "Режим позиции",
            f"Переключить мотор ID {servo_id} с режима скорости на режим позиции?\n"
            "Настройка сохранится после выключения питания.",
            parent=self.root, default=messagebox.NO,
        )
        if confirmed and self.dispatch("position_mode", servo_id):
            self.message(f"ID {servo_id}. Переключаю в режим позиции…")

    def select_available(self):
        motors = [str(servo_id) for servo_id, row in self.rows.items()
                  if row.identity.model in MODELS and self._fresh(row)]
        if motors:
            self.table.selection_set(*motors)
            self._select()

    def change_id(self):
        if len(self.selection) != 1 or self.busy:
            return
        old = self.selection[0]
        new = simpledialog.askinteger(
            "Смена ID",
            f"Текущий ID: {old}. Оставьте на шине только этот мотор и снимите удержание.\n\nВведите новый ID от 0 до {MAX_ID}.",
            parent=self.root, minvalue=0, maxvalue=MAX_ID,
        )
        if new is not None:
            self.dispatch("change_id", old, new)

    def _target_edited(self, *_):
        try:
            value = int(self.target.get())
        except ValueError:
            return
        lower, upper = self.range or (0, MAX_POSITION)
        if lower <= value <= upper:
            self.slider_value.set(value)

    def _slide(self, value):
        self.target.set(str(round(float(value))))

    def _selected_rows(self):
        return [self.rows[servo_id] for servo_id in self.selection if servo_id in self.rows]

    def _select(self, _event=None):
        chosen = tuple(int(item) for item in self.table.selection() if int(item) in self.rows)
        changed = chosen != self.selection
        self.selection = chosen
        self._refresh_controls()
        if changed:
            self.target.set("")
            self._initialize_target()
        self._render_graph()

    def _initialize_target(self):
        chosen = self._selected_rows()
        if chosen and self.range and self._fresh(chosen[0]) and not self.target.get():
            feedback = chosen[0].feedback
            start = min(self.range[1], max(self.range[0], feedback.position))
            self.target.set(str(start))

    def _refresh_controls(self):
        idle = self.connected and self.busy is None
        sweeping = self.sweep_id is not None
        active = idle and not sweeping
        if self.worker is None:
            self.connect_button.configure(text="Подключить")
        elif self.busy == "disconnect":
            self.connect_button.configure(text="Отключаюсь…")
        else:
            self.connect_button.configure(text="Отключить")
        self._enable(self.connect_button, self.busy != "disconnect")
        for widget in (self.demo_button, self.ports_button, self.port_box, self.baud_box):
            self._enable(widget, self.worker is None)
        scanning = self.busy == "scan"
        self.scan_button.configure(text="Остановить поиск" if scanning else "Найти моторы")
        self._enable(self.scan_button, active or (scanning and not self.worker.cancel_scan.is_set()))
        self._enable(self.select_online, active and any(row.identity.model in MODELS and self._fresh(row) for row in self.rows.values()))
        for widget in (self.first_box, self.last_box):
            self._enable(widget, active)

        chosen = self._selected_rows()
        known = bool(chosen) and all(row.identity.model in MODELS for row in chosen)
        has_feedback = known and all(self._fresh(row) for row in chosen)
        can_switch_mode = (known and len(chosen) == 1 and chosen[0].config is not None
                           and chosen[0].config.mode == SPEED_MODE)
        self.range = None
        note = "Выберите мотор в таблице."
        title = "Управление"
        if self.connected and not self.rows:
            note = "Ищу моторы…" if scanning else "Моторы не найдены. Проверьте питание и скорость порта."
        if chosen:
            identity = chosen[0].identity
            title = f"ID {identity.servo_id} · {identity.name}" if len(chosen) == 1 else f"Группа · выбрано {len(chosen)}"
            if not known:
                note = "Управление доступно для моделей ST3215 и ST3250."
            elif any(row.config is None for row in chosen):
                note = "Повторно считываю пределы позиции мотора."
            elif invalid := next((row for row in chosen if not row.config.single_turn), None):
                note = f"ID {invalid.identity.servo_id}. {invalid.config.position_problem}"
            else:
                minimum = max(row.config.minimum for row in chosen)
                maximum = min(row.config.maximum for row in chosen)
                if minimum <= maximum:
                    self.range = (minimum, maximum)
                    note = "Общая позиция для выбранных моторов." if len(chosen) > 1 else "Укажите целевую позицию."
                else:
                    note = "У выбранных моторов нет общей допустимой позиции."
            if known and not has_feedback:
                note = next((row.issue for row in chosen if row.issue), "Ожидаю свежую обратную связь.")
            elif has_feedback and self.range is not None:
                if any(row.feedback.status for row in chosen):
                    note = "Мотор сообщает об ошибке. Проверьте состояние в таблице."
                elif any(row.feedback.torque != 1 for row in chosen):
                    note = "Для отправки позиции включите удержание."
            if can_switch_mode:
                note = "Режим скорости (1)."
        limits = self.range or (0, MAX_POSITION)
        if float(self.slider.cget("from")) != limits[0] or float(self.slider.cget("to")) != limits[1]:
            self.slider.configure(from_=limits[0], to=limits[1])
            self.position_box.configure(from_=limits[0], to=limits[1])
        self.minimum_label.configure(text=str(limits[0]) if self.range else "")
        self.maximum_label.configure(text=str(limits[1]) if self.range else "")
        self.control_title.configure(text=title)
        self.control_note.configure(text=note)
        editing = active and self.range is not None
        for widget in (self.slider, self.position_box, self.speed_box, self.acc_box):
            self._enable(widget, editing)
        take_actual = editing and has_feedback and len(chosen) == 1
        self._enable(self.use_actual, take_actual and limits[0] <= chosen[0].feedback.position <= limits[1])
        self._enable(self.position_mode_button, active and can_switch_mode)
        if can_switch_mode and not self.position_mode_button.winfo_manager():
            self.use_actual.grid_remove()
            self.position_mode_button.grid()
        elif not can_switch_mode and self.position_mode_button.winfo_manager():
            self.position_mode_button.grid_remove()
            self.use_actual.grid()
        may_move = has_feedback and all(row.feedback.torque == 1 and row.feedback.status == 0 for row in chosen)
        self._enable(self.send_button, active and may_move and self.range is not None)
        self._enable(self.torque_on, active and known)
        self._enable(self.torque_off, idle and (sweeping or known))
        self._enable(self.id_button, active and known and len(chosen) == 1)
        self._refresh_sweep_controls(chosen, active, idle, may_move)
        self.selection_label.configure(text=f"Выбрано {len(chosen)} из {len(self.rows)}  ·  Ctrl / Shift для группы"
                                       if self.rows else "Ctrl / Shift для выбора группы")

    def _refresh_sweep_controls(self, chosen, active, idle, may_move):
        available = 0
        if active and len(chosen) == 1 and self.range is not None and self._fresh(chosen[0]):
            position = chosen[0].feedback.position
            available = min(position - self.range[0], self.range[1] - position)
            if available >= 1:
                if float(self.sweep_slider.cget("to")) != available:
                    self.sweep_slider.configure(to=available)
                if self.sweep_amplitude.get() > available:
                    self.sweep_amplitude.set(available)
        self._enable(self.sweep_slider, available >= 1)
        sweeping = self.sweep_id is not None
        self.sweep_button.configure(text="Остановить" if sweeping else "Вперёд-назад")
        self._enable(self.sweep_button, (idle and sweeping) or (available >= 1 and may_move))

    @staticmethod
    def _enable(widget, enabled):
        widget.state(["!disabled"] if enabled else ["disabled"])

    def _fresh(self, row):
        return bool(
            row.feedback and not row.issue
            and monotonic() - row.feedback.timestamp < max(0.9, len(self.rows) * 0.085 + 0.35)
        )

    def _render_row(self, servo_id):
        row = self.rows[servo_id]
        feedback = row.feedback
        live = self._fresh(row)
        tags = ["stripe"] if servo_id % 2 == 0 else []
        if feedback and live:
            delta = feedback.goal - feedback.position
            state = status_text(feedback.status) or ("Движется" if feedback.moving else "На связи")
            torque = {0: "Выкл.", 1: "Вкл.", 2: "Демпфер"}.get(feedback.torque, str(feedback.torque))
            values = (feedback.goal, feedback.position, f"{delta:+d}" if delta else "0", feedback.speed,
                      f"{feedback.voltage:.1f}", feedback.temperature, f"{feedback.current:.2f}", torque, state)
            if feedback.status:
                tags.append("warning")
        else:
            tags.append("stale")
            state = "Нет связи" if row.issue else "Устарели" if feedback else "Ожидание"
            values = ("—",) * 8 + (state,)
        sent = row.motion.position if row.motion else "—"
        self.table.item(str(servo_id), values=(servo_id, row.identity.name, sent, *values), tags=tags)

    def _render_graph(self):
        chosen = self._selected_rows()
        readings = ["—", "—", "—", "—"]
        if len(chosen) != 1:
            self.graph_title.configure(text="Обратная связь")
            self.graph.history = ()
            self.graph.empty_text = "Выберите один мотор в таблице"
            data = ""
        else:
            row = chosen[0]
            self.graph_title.configure(text=f"Обратная связь · ID {row.identity.servo_id}")
            self.graph.history = row.history
            self.graph.empty_text = "Пока нет данных для графика"
            if row.motion:
                readings[0] = str(row.motion.position)
            if self._fresh(row):
                feedback = row.feedback
                delta = feedback.goal - feedback.position
                readings[1:] = [str(feedback.goal), str(feedback.position), f"{delta:+d}" if delta else "0"]
                angle = f"{feedback.position * 360 / 4096:.1f}°  ·  " if row.config and row.config.single_turn else ""
                data = (f"{feedback.voltage:.1f} В  ·  {feedback.current:.2f} А  ·  {feedback.temperature} °C"
                        f"\n{angle}Скорость {feedback.speed}  ·  Нагрузка {feedback.load:.1f}%")
                if feedback.status:
                    data += f"\nОшибка: {status_text(feedback.status)}"
                elif row.motion:
                    data += f"\nКоманда: скорость {row.motion.speed}, ускорение {row.motion.acceleration}"
            else:
                data = row.issue or ("Данные устарели." if row.feedback else "Ожидаю обратную связь.")
        for label, value in zip(self.readings, readings):
            label.configure(text=value)
        self.measurements.configure(text=data)
        self.graph.draw()

    def _refresh_times(self):
        for servo_id in self.rows:
            self._render_row(servo_id)
        self._refresh_controls()
        self._render_graph()
        self.root.after(300, self._refresh_times)

    def _consume_events(self):
        dirty = False
        for _ in range(300):
            try:
                event, args = self.events.get_nowait()
            except Empty:
                break
            if event == "connected":
                self.connected = True
                if not self.worker.stop_requested.is_set():
                    self.busy = None
                    label = "Демо · 3 мотора" if self.worker.demo else "Порт открыт"
                    self.connection.configure(text=label, bg="#d9f0e6", fg=GREEN)
                    self.scan()
            elif event == "disconnected":
                self.connected = False
                self.worker = None
                self.busy = None
                self.sweep_id = None
                self.progress.grid_remove()
                self.connection.configure(text="Отключено", bg="#dce5f1", fg=MUTED)
                if not self.port_failed:
                    self.message("Шина отключена.")
                for servo_id, row in self.rows.items():
                    if not row.issue:
                        row.history.append((monotonic(), None, None))
                    row.issue = "Шина отключена."
                    self._render_row(servo_id)
                if self.closing:
                    self.root.destroy()
                    return
            elif event == "ready":
                if self.busy != "disconnect":
                    self.busy = None
                self.progress.grid_remove()
            elif event == "progress":
                self.progress.configure(value=args[0], maximum=args[1])
            elif event == "device":
                self._on_device(*args)
            elif event == "renamed":
                old, new = args
                del self.rows[old]
                self.table.delete(str(old))
                self.table.selection_set(str(new))
                self.table.see(str(new))
                self._select()
            elif event == "missing":
                servo_id, issue = args
                row = self.rows[servo_id]
                if not row.issue:
                    row.history.append((monotonic(), None, None))
                    self._log(issue, True)
                row.issue = issue
                self._render_row(servo_id)
            elif event == "feedback":
                servo_id, feedback = args
                row = self.rows[servo_id]
                row.feedback = feedback
                row.issue = ""
                row.history.append((feedback.timestamp, feedback.goal, feedback.position))
                self._render_row(servo_id)
            elif event == "motion":
                servo_id, motion = args
                self.rows[servo_id].motion = motion
                self._render_row(servo_id)
            elif event == "sweep":
                self.sweep_id, = args
            elif event == "message":
                self.message(*args)
            elif event == "port_error":
                self.port_failed = True
                self.message(args[0], True)
            elif event == "log":
                self._log(*args)
            dirty = True
        if dirty:
            self.table_title.configure(text=f"Моторы · {len(self.rows)}" if self.rows else "Моторы")
            self._refresh_controls()
            self._initialize_target()
        self.root.after(35, self._consume_events)

    def _on_device(self, identity, config):
        servo_id = identity.servo_id
        previous = self.rows.get(servo_id)
        if previous is None or previous.identity != identity:
            self.rows[servo_id] = Row(identity, config)
        else:
            previous.config = config
        if previous is None:
            index = sum(other < servo_id for other in self.rows)
            self.table.insert("", index, iid=str(servo_id))
            if not self.selection and len(self.rows) == 1:
                self.table.selection_set(str(servo_id))
                self._select()
        self._render_row(servo_id)

    def message(self, text, error=False):
        first_line = text.splitlines()[0]
        if len(first_line) > 115:
            first_line = first_line[:112] + "…"
        self.status.configure(text=first_line, fg=ERROR if error else MUTED)
        self._log(text, error)

    def _log(self, text, error=False):
        stamp = datetime.now().strftime("%H:%M:%S")
        self.logs.append(f"{stamp}   {'Ошибка   ' if error else ''}{text}")
        if self.log_window is not None and self.log_window.winfo_exists():
            self._render_log()

    def open_log(self):
        if self.log_window is not None and self.log_window.winfo_exists():
            self.log_window.lift()
            return
        self.log_window = tk.Toplevel(self.root)
        self.log_window.title("Журнал шины Feetech")
        self.log_window.geometry("720x420")
        self.log_window.rowconfigure(0, weight=1)
        self.log_window.columnconfigure(0, weight=1)
        self.log_text = tk.Text(self.log_window, padx=12, pady=10, wrap="word", bg=PAPER, fg=INK, font="TkTextFont")
        self.log_text.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(self.log_window, command=self.log_text.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=scroll.set)
        self._render_log()

    def _render_log(self):
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.insert("1.0", "\n".join(self.logs))
        self.log_text.configure(state="disabled")
        self.log_text.see("end")
