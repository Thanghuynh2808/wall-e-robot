#!/usr/bin/env python3
"""
map_waypoint_picker.py
----------------------
GUI tool: Load bản đồ ROS (.pgm + .yaml), click để chọn waypoints A→B→C,
tự động tạo lệnh chạy multi_waypoint_nav.py.

Cách dùng (trên Ubuntu):
    python3 map_waypoint_picker.py
    python3 map_waypoint_picker.py --map /path/to/map2.yaml

Yêu cầu: python3-tk (thường đã có sẵn)
    sudo apt install python3-tk python3-pil python3-pil.imagetk
"""

import argparse
import math
import os
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

# ── Thử import PIL, nếu không có thì dùng pgm thủ công ──
try:
    from PIL import Image, ImageTk, ImageDraw, ImageFilter
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

import yaml


# ════════════════════════════════════════════════════════════
# Đọc bản đồ
# ════════════════════════════════════════════════════════════

def load_map_yaml(yaml_path: str) -> dict:
    """Đọc file .yaml của ROS map, trả về dict với image, resolution, origin."""
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    # image path có thể relative hoặc absolute
    img_path = data.get('image', '')
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(yaml_path), img_path)
    data['image_path'] = img_path
    return data


def load_pgm_manual(path: str):
    """Load file PGM (P5 binary) mà không cần PIL. Trả về list of list pixels."""
    with open(path, 'rb') as f:
        # Đọc header
        magic = f.readline().decode().strip()
        assert magic in ('P5', 'P2'), f"Không phải PGM: {magic}"
        # Bỏ comment
        line = f.readline().decode().strip()
        while line.startswith('#'):
            line = f.readline().decode().strip()
        width, height = map(int, line.split())
        maxval = int(f.readline().decode().strip())
        # Đọc data
        if magic == 'P5':
            raw = f.read()
            pixels = []
            for y in range(height):
                row = []
                for x in range(width):
                    row.append(raw[y * width + x])
                pixels.append(row)
        else:  # P2 ASCII
            vals = f.read().decode().split()
            pixels = []
            idx = 0
            for y in range(height):
                row = [int(vals[idx + x]) for x in range(width)]
                pixels.append(row)
                idx += width
    return width, height, maxval, pixels


# ════════════════════════════════════════════════════════════
# Conversion: pixel ↔ map coordinates (meters)
# ════════════════════════════════════════════════════════════

def pixel_to_map(px, py, img_height, resolution, origin):
    """
    Convert pixel (px, py) → map coordinates (x, y) in meters.
    ROS convention: origin = [x0, y0, yaw], pixel (0,0) = top-left.
    map_y = origin[1] + (img_height - 1 - py) * resolution
    map_x = origin[0] + px * resolution
    """
    mx = origin[0] + px * resolution
    my = origin[1] + (img_height - 1 - py) * resolution
    return mx, my


def map_to_pixel(mx, my, img_height, resolution, origin):
    """Convert map coordinates (x, y) → pixel (px, py)."""
    px = int((mx - origin[0]) / resolution)
    py = int(img_height - 1 - (my - origin[1]) / resolution)
    return px, py


# ════════════════════════════════════════════════════════════
# App chính
# ════════════════════════════════════════════════════════════

WAYPOINT_COLORS = ['#FF4444', '#44BB44', '#4488FF', '#FF8800', '#BB44FF', '#00CCCC']
WAYPOINT_LABELS = 'ABCDEFGHIJ'


class MapWaypointPicker:
    def __init__(self, root: tk.Tk, map_yaml_path: str = None):
        self.root = root
        self.root.title("Wall-E Map Waypoint Picker")
        self.root.configure(bg='#1a1a2e')

        # State
        self.map_info = None       # dict từ yaml
        self.img_width = 0
        self.img_height = 0
        self.scale = 1.0           # zoom scale
        self.waypoints = []        # list of (map_x, map_y, yaw_deg, label)
        self.mode = 'waypoint'     # 'waypoint' hoặc 'init'
        self.init_pose = None      # (x, y, yaw)
        self.drag_start = None
        self.canvas_offset = [0, 0]  # pan offset
        self.tk_image = None
        self.base_pil_image = None   # original PIL image

        self._build_ui()

        if map_yaml_path:
            self._load_map(map_yaml_path)

    # ── UI ──────────────────────────────────────────────────

    def _build_ui(self):
        # ── Toolbar ──
        toolbar = tk.Frame(self.root, bg='#16213e', pady=6)
        toolbar.pack(fill='x')

        btn_style = {'bg': '#0f3460', 'fg': 'white', 'relief': 'flat',
                     'padx': 12, 'pady': 6, 'cursor': 'hand2',
                     'font': ('Consolas', 10, 'bold')}

        tk.Button(toolbar, text='📂 Load Map', command=self._open_map, **btn_style).pack(side='left', padx=4)
        tk.Button(toolbar, text='🏁 Set Init Pose', command=self._mode_init, **btn_style).pack(side='left', padx=4)
        tk.Button(toolbar, text='📍 Add Waypoint', command=self._mode_waypoint, **btn_style).pack(side='left', padx=4)
        tk.Button(toolbar, text='↩ Undo Last', command=self._undo, **btn_style).pack(side='left', padx=4)
        tk.Button(toolbar, text='🗑 Clear All', command=self._clear_all, **btn_style).pack(side='left', padx=4)
        tk.Button(toolbar, text='▶ Generate Command', command=self._generate_command,
                  bg='#e94560', fg='white', relief='flat', padx=12, pady=6,
                  cursor='hand2', font=('Consolas', 10, 'bold')).pack(side='right', padx=8)

        # ── Status bar ──
        self.status_var = tk.StringVar(value='Nhấn "Load Map" để bắt đầu.')
        status_bar = tk.Label(self.root, textvariable=self.status_var,
                              bg='#0f3460', fg='#aaaaff', anchor='w',
                              font=('Consolas', 9), padx=8)
        status_bar.pack(fill='x')

        # ── Main area ──
        main = tk.Frame(self.root, bg='#1a1a2e')
        main.pack(fill='both', expand=True)

        # Canvas
        self.canvas = tk.Canvas(main, bg='#0d0d1a', cursor='crosshair',
                                highlightthickness=0)
        self.canvas.pack(side='left', fill='both', expand=True)
        self.canvas.bind('<Button-1>', self._on_click)
        self.canvas.bind('<Button-3>', self._on_right_click)
        self.canvas.bind('<MouseWheel>', self._on_scroll)
        self.canvas.bind('<Button-4>', self._on_scroll)   # Linux scroll up
        self.canvas.bind('<Button-5>', self._on_scroll)   # Linux scroll down
        self.canvas.bind('<ButtonPress-2>', self._pan_start)
        self.canvas.bind('<B2-Motion>', self._pan_move)
        self.canvas.bind('<Motion>', self._on_mouse_move)

        # ── Right panel ──
        right = tk.Frame(main, bg='#16213e', width=280)
        right.pack(side='right', fill='y')
        right.pack_propagate(False)

        tk.Label(right, text='WAYPOINTS', bg='#16213e', fg='#e94560',
                 font=('Consolas', 11, 'bold'), pady=8).pack()

        # Waypoint list
        list_frame = tk.Frame(right, bg='#16213e')
        list_frame.pack(fill='both', expand=True, padx=8)

        self.wp_listbox = tk.Listbox(list_frame, bg='#0d0d1a', fg='white',
                                     font=('Consolas', 9), selectbackground='#e94560',
                                     relief='flat', borderwidth=0, height=12)
        self.wp_listbox.pack(fill='both', expand=True)

        # Delay control
        delay_frame = tk.Frame(right, bg='#16213e', pady=6)
        delay_frame.pack(fill='x', padx=8)
        tk.Label(delay_frame, text='Delay giữa điểm (giây):', bg='#16213e',
                 fg='#aaaaff', font=('Consolas', 9)).pack(anchor='w')
        self.delay_var = tk.StringVar(value='5')
        tk.Entry(delay_frame, textvariable=self.delay_var, bg='#0f3460', fg='white',
                 font=('Consolas', 11), relief='flat', width=8).pack(anchor='w', pady=2)

        # Yaw control
        yaw_frame = tk.Frame(right, bg='#16213e', pady=4)
        yaw_frame.pack(fill='x', padx=8)
        tk.Label(yaw_frame, text='Hướng điểm tiếp theo (°):', bg='#16213e',
                 fg='#aaaaff', font=('Consolas', 9)).pack(anchor='w')
        self.yaw_var = tk.StringVar(value='0')
        yaw_entry = tk.Entry(yaw_frame, textvariable=self.yaw_var, bg='#0f3460',
                             fg='white', font=('Consolas', 11), relief='flat', width=8)
        yaw_entry.pack(anchor='w', pady=2)
        tk.Label(yaw_frame, text='0°=Đông  90°=Bắc  180°=Tây  -90°=Nam',
                 bg='#16213e', fg='#666699', font=('Consolas', 8)).pack(anchor='w')

        # Output box
        tk.Label(right, text='LỆNH CHẠY:', bg='#16213e', fg='#e94560',
                 font=('Consolas', 10, 'bold'), pady=4).pack()
        self.cmd_text = tk.Text(right, bg='#0d0d1a', fg='#44ff88', height=8,
                                font=('Consolas', 8), relief='flat', wrap='word',
                                padx=4, pady=4)
        self.cmd_text.pack(fill='x', padx=8, pady=(0, 4))

        tk.Button(right, text='📋 Copy Lệnh', command=self._copy_command,
                  bg='#0f3460', fg='white', relief='flat', padx=8, pady=4,
                  cursor='hand2', font=('Consolas', 9, 'bold')).pack(pady=4)

        # Mode indicator
        self.mode_label = tk.Label(right, text='MODE: ADD WAYPOINT',
                                   bg='#44BB44', fg='white',
                                   font=('Consolas', 9, 'bold'), pady=4)
        self.mode_label.pack(fill='x', padx=8, pady=4)

        # Coord display
        self.coord_var = tk.StringVar(value='Hover: --')
        tk.Label(right, textvariable=self.coord_var, bg='#16213e', fg='#aaaaff',
                 font=('Consolas', 8)).pack(pady=2)

    # ── Map loading ─────────────────────────────────────────

    def _open_map(self):
        path = filedialog.askopenfilename(
            title='Chọn file map .yaml',
            filetypes=[('YAML files', '*.yaml'), ('All files', '*.*')],
            initialdir=os.path.expanduser('~')
        )
        if path:
            self._load_map(path)

    def _load_map(self, yaml_path: str):
        try:
            self.map_info = load_map_yaml(yaml_path)
            img_path = self.map_info['image_path']

            if HAS_PIL:
                pil_img = Image.open(img_path).convert('RGB')
                # Tô màu: đen=vật cản, trắng=tự do, xám=unknown
                arr = list(pil_img.getdata())
                colored = []
                for r, g, b in arr:
                    gray = (r + g + b) // 3
                    if gray < 50:       # Vật cản (đen) → đỏ đậm
                        colored.append((180, 40, 40))
                    elif gray > 200:    # Tự do (trắng) → xanh nhạt
                        colored.append((230, 245, 255))
                    else:               # Unknown (xám) → xám trung
                        colored.append((120, 120, 140))
                pil_img.putdata(colored)
                self.base_pil_image = pil_img
                self.img_width, self.img_height = pil_img.size
            else:
                # Fallback: dùng pgm thủ công
                w, h, maxval, pixels = load_pgm_manual(img_path)
                self.img_width, self.img_height = w, h
                self.base_pil_image = None
                self._pixels_raw = pixels

            # Tính scale ban đầu để vừa màn hình
            canvas_w = self.canvas.winfo_width() or 700
            canvas_h = self.canvas.winfo_height() or 600
            sx = canvas_w / self.img_width
            sy = canvas_h / self.img_height
            self.scale = min(sx, sy, 2.0)
            self.canvas_offset = [0, 0]

            self._render_map()
            self.status_var.set(
                f'✓ Map loaded: {os.path.basename(yaml_path)} | '
                f'{self.img_width}×{self.img_height}px | '
                f'res={self.map_info["resolution"]}m/px'
            )
            self.waypoints.clear()
            self.init_pose = None
            self._update_list()

        except Exception as e:
            messagebox.showerror('Lỗi', f'Không load được map:\n{e}')

    def _render_map(self):
        if self.base_pil_image is None:
            return
        # Scale image
        new_w = max(1, int(self.img_width * self.scale))
        new_h = max(1, int(self.img_height * self.scale))
        resized = self.base_pil_image.resize((new_w, new_h), Image.NEAREST)

        # Vẽ waypoints lên ảnh
        draw = ImageDraw.Draw(resized)
        res = self.map_info['resolution']
        origin = self.map_info['origin']

        # Init pose
        if self.init_pose:
            px, py = map_to_pixel(self.init_pose[0], self.init_pose[1],
                                  self.img_height, res, origin)
            sx, sy = int(px * self.scale), int(py * self.scale)
            r = max(3, int(3 * self.scale))
            draw.ellipse([sx-r, sy-r, sx+r, sy+r], fill='#FFD700', outline='white', width=1)
            draw.text((sx+r+2, sy-6), 'START', fill='#FFD700')

        # Waypoints + lines
        prev = None
        for i, (mx, my, yaw, lbl) in enumerate(self.waypoints):
            px, py = map_to_pixel(mx, my, self.img_height, res, origin)
            sx, sy = int(px * self.scale), int(py * self.scale)
            color = WAYPOINT_COLORS[i % len(WAYPOINT_COLORS)]
            r = max(3, int(3 * self.scale))
            if prev:
                draw.line([prev[0], prev[1], sx, sy], fill='#ffffff', width=1)
            draw.ellipse([sx-r, sy-r, sx+r, sy+r], fill=color, outline='white', width=1)
            draw.text((sx+r+2, sy-6), lbl, fill=color)
            prev = (sx, sy)

        self.tk_image = ImageTk.PhotoImage(resized)
        self.canvas.delete('all')
        ox, oy = self.canvas_offset
        self.canvas.create_image(ox, oy, anchor='nw', image=self.tk_image)

    # ── Interaction ─────────────────────────────────────────

    def _mode_waypoint(self):
        self.mode = 'waypoint'
        self.mode_label.config(text='MODE: ADD WAYPOINT', bg='#44BB44')
        self.status_var.set('Click trái trên bản đồ để thêm waypoint. Click phải = xoay hướng.')

    def _mode_init(self):
        self.mode = 'init'
        self.mode_label.config(text='MODE: SET INIT POSE', bg='#FF8800')
        self.status_var.set('Click trái để đặt vị trí ban đầu (START) của robot.')

    def _on_click(self, event):
        if self.map_info is None:
            messagebox.showinfo('Chú ý', 'Hãy load map trước!')
            return
        # Convert canvas → image pixel
        ox, oy = self.canvas_offset
        img_px = (event.x - ox) / self.scale
        img_py = (event.y - oy) / self.scale
        if not (0 <= img_px < self.img_width and 0 <= img_py < self.img_height):
            return
        # Check obstacle (nếu PIL)
        if self.base_pil_image:
            r, g, b = self.base_pil_image.getpixel((int(img_px), int(img_py)))
            if r > 150 and g < 80:  # màu đỏ = vật cản
                self.status_var.set('⚠️ Điểm này nằm trong vật cản! Chọn vùng sáng hơn.')
                return

        mx, my = pixel_to_map(img_px, img_py,
                               self.img_height,
                               self.map_info['resolution'],
                               self.map_info['origin'])
        try:
            yaw = float(self.yaw_var.get())
        except ValueError:
            yaw = 0.0

        if self.mode == 'init':
            self.init_pose = (mx, my, yaw)
            self.status_var.set(f'✓ Init pose: x={mx:.3f}, y={my:.3f}, yaw={yaw:.1f}°')
            self._render_map()
        else:
            if len(self.waypoints) >= 10:
                messagebox.showinfo('Giới hạn', 'Tối đa 10 waypoints!')
                return
            lbl = WAYPOINT_LABELS[len(self.waypoints)]
            self.waypoints.append((mx, my, yaw, lbl))
            self.status_var.set(
                f'✓ Điểm {lbl}: x={mx:.3f}m, y={my:.3f}m, yaw={yaw:.1f}°'
            )
            self._update_list()
            self._render_map()

    def _on_right_click(self, event):
        """Right click: xoay hướng điểm gần nhất."""
        pass  # Tính năng mở rộng

    def _on_mouse_move(self, event):
        if self.map_info is None:
            return
        ox, oy = self.canvas_offset
        img_px = (event.x - ox) / self.scale
        img_py = (event.y - oy) / self.scale
        if 0 <= img_px < self.img_width and 0 <= img_py < self.img_height:
            mx, my = pixel_to_map(img_px, img_py,
                                   self.img_height,
                                   self.map_info['resolution'],
                                   self.map_info['origin'])
            self.coord_var.set(f'Hover: x={mx:.3f}m, y={my:.3f}m')

    def _on_scroll(self, event):
        # Zoom
        if event.num == 4 or event.delta > 0:
            factor = 1.15
        else:
            factor = 1 / 1.15
        self.scale = max(0.1, min(10.0, self.scale * factor))
        self._render_map()

    def _pan_start(self, event):
        self.drag_start = (event.x, event.y)

    def _pan_move(self, event):
        if self.drag_start:
            dx = event.x - self.drag_start[0]
            dy = event.y - self.drag_start[1]
            self.canvas_offset[0] += dx
            self.canvas_offset[1] += dy
            self.drag_start = (event.x, event.y)
            self._render_map()

    def _undo(self):
        if self.waypoints:
            removed = self.waypoints.pop()
            self.status_var.set(f'↩ Đã xóa điểm {removed[3]}')
            self._update_list()
            self._render_map()

    def _clear_all(self):
        if messagebox.askyesno('Xác nhận', 'Xóa tất cả waypoints?'):
            self.waypoints.clear()
            self.init_pose = None
            self._update_list()
            self._render_map()

    def _update_list(self):
        self.wp_listbox.delete(0, 'end')
        for i, (mx, my, yaw, lbl) in enumerate(self.waypoints):
            self.wp_listbox.insert('end', f'[{lbl}] x={mx:.3f} y={my:.3f} yaw={yaw:.0f}°')
            self.wp_listbox.itemconfig(i, fg=WAYPOINT_COLORS[i % len(WAYPOINT_COLORS)])

    # ── Generate command ─────────────────────────────────────

    def _generate_command(self):
        if not self.waypoints:
            messagebox.showinfo('Chú ý', 'Chưa có waypoint nào!')
            return
        try:
            delay = float(self.delay_var.get())
        except ValueError:
            delay = 5.0

        lines = []

        # Lệnh set init pose
        if self.init_pose:
            ix, iy, iyaw = self.init_pose
            lines.append('# 1. Set vị trí ban đầu:')
            lines.append(f'python3 set_initial_pose.py --x {ix:.4f} --y {iy:.4f} --yaw {iyaw:.1f}')
            lines.append('')

        # Lệnh navigate
        wp_parts = ' | '.join(
            f'{mx:.4f},{my:.4f},{yaw:.1f}' for mx, my, yaw, _ in self.waypoints
        )
        lines.append('# 2. Chạy navigation:')
        lines.append(f'python3 multi_waypoint_nav.py \\')
        lines.append(f'    --waypoints "{wp_parts}" \\')
        lines.append(f'    --delay {delay:.1f}')

        # One-liner
        lines.append('')
        lines.append('# Hoặc 1 dòng:')
        oneliner = (
            f'python3 multi_waypoint_nav.py --waypoints "{wp_parts}" --delay {delay:.1f}'
        )
        lines.append(oneliner)

        cmd_str = '\n'.join(lines)
        self.cmd_text.delete('1.0', 'end')
        self.cmd_text.insert('end', cmd_str)
        self.status_var.set('✓ Đã tạo lệnh! Nhấn "Copy Lệnh" để copy.')

    def _copy_command(self):
        content = self.cmd_text.get('1.0', 'end').strip()
        if content:
            self.root.clipboard_clear()
            self.root.clipboard_append(content)
            self.status_var.set('✓ Đã copy lệnh vào clipboard!')


# ════════════════════════════════════════════════════════════
# Main
# ════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description='Visual waypoint picker từ ROS map')
    parser.add_argument('--map', type=str, default=None,
                        help='Đường dẫn đến file map.yaml (mặc định: tự chọn)')
    args = parser.parse_args()

    # Tìm map tự động nếu không chỉ định
    map_path = args.map
    if not map_path:
        # Thử tìm trong thư mục mặc định của project
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_dir = os.path.dirname(script_dir)
        candidates = [
            os.path.join(project_dir, 'maps', 'map2.yaml'),
            os.path.join(project_dir, 'maps', 'map.yaml'),
            os.path.expanduser('~/maps/map2.yaml'),
        ]
        for c in candidates:
            if os.path.exists(c):
                map_path = c
                print(f'[Auto] Found map: {map_path}')
                break

    if not HAS_PIL:
        print('[WARN] PIL không có sẵn. Cài đặt bằng:')
        print('  sudo apt install python3-pil python3-pil.imagetk')
        print('  hoặc: pip3 install Pillow')
        print('Tiếp tục với chức năng giới hạn...\n')

    root = tk.Tk()
    root.geometry('1100x700')
    root.minsize(800, 550)
    app = MapWaypointPicker(root, map_yaml_path=map_path)
    root.mainloop()


if __name__ == '__main__':
    main()
