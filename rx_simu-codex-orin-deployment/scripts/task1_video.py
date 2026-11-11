"""Render one replay of visualize_task1_field.py as an MP4 (top-down map + status panel).

Used by `visualize_task1_field.py --video`. Needs matplotlib and an ffmpeg binary with
libx264: $TASK1_FFMPEG, else imageio-ffmpeg's bundled binary, else `ffmpeg` on PATH.
"""
import math
import os
import shutil
import subprocess
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.patches import Circle, Polygon  # noqa: E402

plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Noto Sans CJK SC', 'WenQuanYi Zen Hei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

LAMP = {0: '#7b858f', 1: '#e0362c', 2: '#1fae4b', 3: '#2f6fe8', 4: '#2f6fe8', 5: '#9aa3ad'}
STATE_NAME = ['灭', '闪红', '闪绿', '闪蓝(入口)', '常蓝(出口)', '未识别']
STATE_CN = dict(WAIT_START='等待开始', SEARCH_ENTRY='寻找入口', SEARCH_EXIT='寻找出口', PLAN='规划', LOOK='停船识色',
                ALIGN='对准缝隙', THREAD='穿越缝隙', TRANSIT='航行', APPROACH_ENTRY='驶向入口',
                APPROACH_EXIT='驶向出口', CIRCLE_ENTRY='顺时针绕入口', CIRCLE_EXIT='逆时针绕出口', HOLD='暂停',
                COMPLETE='完成', ABORT='中止')
PHASE_COLOR = dict(SEARCH_ENTRY='#8a96a3', SEARCH_EXIT='#8a96a3', PLAN='#8a96a3', LOOK='#8b5cf6', ALIGN='#d69e2e',
                   THREAD='#dd6b20', TRANSIT='#319795', APPROACH_ENTRY='#319795', APPROACH_EXIT='#319795',
                   CIRCLE_ENTRY='#3182ce', CIRCLE_EXIT='#3182ce', HOLD='#c23a2b', COMPLETE='#2c8a4b',
                   ABORT='#c23a2b')


def ffmpeg():
    if os.environ.get('TASK1_FFMPEG'):
        return os.environ['TASK1_FFMPEG']
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError:
        found = shutil.which('ffmpeg')
        if not found:
            raise RuntimeError('no ffmpeg: set TASK1_FFMPEG or pip install imageio-ffmpeg')
        return found


def lit(state, t):
    return state == 4 or (1 <= state <= 3 and math.floor(t) % 2 == 0)


def event_text(e, names):
    def n(key):
        value = e.get(key)
        return f'{names.get(value, value)}号' if value is not None else '?'
    kind = e['type']
    texts = dict(
        started='任务开始', entry_found=f'找到闪蓝入口 {n("id")}', exit_found=f'找到常蓝出口 {n("id")}',
        circle_entry_done=f'顺时针绕入口 {n("id")} 完成，开始计分', circle_exit_done=f'逆时针绕出口完成',
        credit=(f'计分：{"绿灯在左舷" if e.get("item") == "green_to_port" else "红灯在右舷"}（{n("id")}）'),
        gap_passed=f'穿过缝隙：左舷 {n("port")} | 右舷 {n("starboard")}', wrong_side=f'错侧：{n("id")}',
        classified=f'{n("id")} 识别为 {STATE_NAME[e["state"]] if isinstance(e.get("state"), int) else e.get("state")}',
        complete='任务完成', abort=f'中止：{e.get("reason", "")}', hold='输入超时，暂停', resumed='恢复',
        survey_done='所有浮标已识别，去出口', look_from_afar=f'远距离识别 {n("id")}')
    return texts.get(kind)


def latest(items, t):
    found = None
    for item in items:
        if item[0] > t:
            break
        found = item
    return found


def render(data, run, path, fps=30, speed=10.0, size=(1280, 720), hold_end=4.0):
    """Write `path` (MP4): `speed` seconds of mission per second of video."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    field, frames, names = data['field'], run['frames'], run['names']
    corners = [field['corners'][k] for k in 'ADCB']
    xs = [p[0] for p in corners]+[f[1] for f in frames]
    ys = [p[1] for p in corners]+[f[2] for f in frames]
    x0, x1, y0, y1 = min(xs)-1.5, max(xs)+1.5, min(ys)-1.5, max(ys)+1.5

    dpi = 100
    fig = plt.figure(figsize=(size[0]/dpi, size[1]/dpi), dpi=dpi, facecolor='#f7f8fa')
    ax = fig.add_axes([0.01, 0.02, 0.56, 0.96])
    panel = fig.add_axes([0.58, 0.02, 0.41, 0.96])
    panel.axis('off')
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect('equal')
    ax.set_facecolor('#e8f1f8')
    ax.tick_params(labelsize=7, colors='#6b7480')
    ax.add_patch(Polygon(corners, closed=True, fill=False, ls='--', lw=1.0, ec='#6b7480'))
    for structure in field['structures']:
        ax.add_patch(Polygon(structure, closed=True, fc='#b9a88f', ec='#7d6b52', lw=0.8))
    for k, p in field['corners'].items():
        ax.add_patch(Circle(p, 0.3, fc='#cfd4da', ec='#4a5560', lw=0.8))
        ax.text(p[0]+0.45, p[1]+0.45, k, fontsize=8, color='#4a5560')
    for k, p in field.get('markers', {}).items():
        ax.add_patch(Circle(p, 0.26, fc='#cfd4da', ec='#4a5560', lw=0.8))
        ax.text(p[0]+0.4, p[1]-0.9, '起点门', fontsize=7, color='#4a5560')
    lamps = {}
    for b in run['truth']:
        lamps[b['id']] = (ax.add_patch(Circle((b['x'], b['y']), 0.26, fc=LAMP[b['state']], ec='#2d3640', lw=0.8)),
                          b['state'])
        ax.text(b['x']+0.35, b['y']+0.35, b['id'], fontsize=8, color='#2d3640', weight='bold')
    rings = []
    trail = LineCollection([], linewidths=2.0)
    ax.add_collection(trail)
    boat = ax.add_patch(Polygon(np.zeros((5, 2)), closed=True, fc='#ffb020', ec='#6b4a00', lw=1.0, zorder=5))
    target, = ax.plot([], [], marker='+', color='#ffb020', ms=10, mew=1.5, zorder=4)
    title = panel.text(0.0, 0.985, data['meta']['title'], fontsize=13, weight='bold', va='top', color='#1f2933')
    name = panel.text(0.0, 0.935, run['name'], fontsize=9, va='top', color='#3e4c59', wrap=True)
    clock = panel.text(0.0, 0.86, '', fontsize=12, va='top', color='#1f2933')
    state = panel.text(0.0, 0.815, '', fontsize=12, va='top', weight='bold')
    detail = panel.text(0.0, 0.77, '', fontsize=8.5, va='top', color='#52606d')
    log = panel.text(0.0, 0.715, '', fontsize=9, va='top', color='#1f2933', linespacing=1.55)
    score = panel.text(0.0, 0.30, '', fontsize=9.5, va='top', color='#1f2933', linespacing=1.5)
    legend = ('图例：彩色实心 = 真实浮标（红/绿/蓝灯闪烁），圆环 = 船上地图（虚线 = 未识别），'
              '轨迹颜色 = 阶段，灰色块 = 码头。')
    panel.text(0.0, 0.035, legend, fontsize=7.5, va='bottom', color='#6b7480', wrap=True)
    del title, name

    seen, classified = {}, []
    for when, snap in run['maps']:  # Lamp classifications from the vessel's map.
        for m_id, _, _, st in snap:
            if st != 5 and seen.get(m_id) != st:
                classified.append(dict(type='classified', id=m_id, state=st, time=when))
            seen[m_id] = st
    events = sorted([e for e in run['events'] if event_text(e, names)]+classified, key=lambda e: e['time'])
    track = np.array([[f[1], f[2]] for f in frames])
    colours = [PHASE_COLOR.get(f[6], '#8a96a3') for f in frames]
    end = frames[-1][0]
    count = int(end/speed*fps)+int(hold_end*fps)+1
    sc = run['verdict']['score']
    it = sc['items']
    summary = (f'Core 估分 {sc["total"]} / {sc["possible"]}（心跳按已得计）\n'
               f'上报入口/出口 {it["report_entry"]+it["report_exit"]} / 20\n'
               f'上报绿/红位置 {it["report_green"]} + {it["report_red"]}\n'
               f'顺时针绕入口 {it["circle_entry"]}，逆时针绕出口 {it["circle_exit"]}\n'
               f'只左舷过绿 {it["green_only_port"]}，只右舷过红 {it["red_only_starboard"]}\n'
               f'全成奖励 {it["bonus"]}，碰撞 {it["collisions"]}')

    command = [ffmpeg(), '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
               '-s', f'{size[0]}x{size[1]}', '-r', str(fps), '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
               '-crf', '23', '-movflags', '+faststart', str(path)]
    encoder = subprocess.Popen(command, stdin=subprocess.PIPE)
    index = 0
    try:
        for k in range(count):
            t = min(end, k*speed/fps)
            while index+1 < len(frames) and frames[index+1][0] <= t:
                index += 1
            f = frames[index]
            for b_id, (patch, st) in lamps.items():
                patch.set_facecolor(LAMP[st] if lit(st, t) or st == 0 else '#ffffff')
            if index > 0:
                segments = np.stack([track[:index], track[1:index+1]], axis=1)
                trail.set_segments(segments)
                trail.set_colors(colours[:index])
            x, y, yaw = f[1], f[2], f[3]
            c, s = math.cos(yaw), math.sin(yaw)
            shape = [(0.66, 0), (0.3, -0.35), (-0.54, -0.35), (-0.54, 0.35), (0.3, 0.35)]
            boat.set_xy([(x+px*c-py*s, y+px*s+py*c) for px, py in shape])
            target.set_data(([f[8][0]], [f[8][1]]) if f[8] else ([], []))
            for ring in rings:
                ring.remove()
            rings = []
            snap = latest(run['maps'], t)
            for _, mx, my, st in (snap[1] if snap else []):
                rings.append(ax.add_patch(Circle((mx, my), 0.62, fill=False, lw=1.4,
                                                 ec=LAMP[st] if st != 5 else '#7b858f', ls='--' if st == 5 else '-')))
            clock.set_text(f'任务时间 {f[0]:6.1f} s    回放 {speed:g}× 速度')
            state.set_text(STATE_CN.get(f[6], f[6]))
            state.set_color(PHASE_COLOR.get(f[6], '#1f2933'))
            detail.set_text(run['details'][f[7]][:70])
            recent = [e for e in events if e['time'] <= t][-9:]
            log.set_text('\n'.join(f'{e["time"]:6.1f} s  {event_text(e, names)}' for e in recent))
            score.set_text(summary if t >= end else '')
            fig.canvas.draw()
            encoder.stdin.write(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].tobytes())
    finally:
        encoder.stdin.close()
        encoder.wait()
        plt.close(fig)
    if encoder.returncode:
        raise RuntimeError(f'ffmpeg failed with {encoder.returncode}')
    return path
