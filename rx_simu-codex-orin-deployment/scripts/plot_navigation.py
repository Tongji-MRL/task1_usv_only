#!/usr/bin/env python3
"""Plot recorded Nav2 references, VRX/EKF tracks, goals and actual costmaps."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.font_manager import FontProperties, fontManager
from matplotlib.patches import Circle, Patch, Polygon, Rectangle
import numpy as np

BLUE, GREEN, ORANGE, PURPLE = '#2563eb', '#07816f', '#d97706', '#7c3aed'
FONT = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
if FONT.exists():
    fontManager.addfont(str(FONT))
    plt.rcParams['font.family'] = FontProperties(fname=str(FONT)).get_name()
plt.rcParams.update({'axes.unicode_minus': False, 'font.size': 10,
                     'axes.spines.top': False, 'axes.spines.right': False,
                     'axes.titleweight': 'bold', 'figure.facecolor': 'white'})
WATER = ListedColormap(['#d7dce1', '#edf7fc', '#39424e'])
cost_colors = plt.get_cmap('YlOrRd')(np.linspace(0.1, 0.9, 101))
cost_colors[:, 3] = 0.72
cost_colors[0] = [0, 0, 0, 0]
cost_colors[99] = matplotlib.colors.to_rgba('#a62691', 0.85)
cost_colors[100] = matplotlib.colors.to_rgba('#272d36', 1)
COST = ListedColormap(cost_colors)


def read_lines(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line]


class Evidence:
    def __init__(self, run, label):
        self.run, self.label = run.resolve(), label
        self.output = self.run / 'visualization'
        self.raw = self.output / 'raw'
        self.events = read_lines(self.raw / 'events.jsonl')
        self.trace = read_lines(self.run / (label + '.jsonl'))
        self.rows = [r for r in self.trace if r.get('phase') in ('navigation', 'post_goal') and 'truth' in r]
        self.active = [r for r in self.rows if r['phase'] == 'navigation']
        if not self.active:
            raise ValueError('No navigation trace')
        self.result = json.loads((self.run / (label + '_result.json')).read_text())
        goals = self.run / 'visualization_goals.json'
        self.goals = json.loads(goals.read_text()) if goals.exists() else [self.result['target']]
        self.target = np.asarray(self.goals[-1])
        self.scene = json.loads((self.run / 'generated/scene.json').read_text())
        self.area = json.loads((self.run / 'generated/water_area.json').read_text())
        self.offset = np.asarray(json.loads((self.run / 'map_alignment.json').read_text())['offset_xy_m'])
        self.initial = self.active[0]
        self.t0 = self.initial['sim']
        self.truth = np.asarray([r['truth'] for r in self.active])
        self.ekf = np.asarray([r.get('ekf', [np.nan] * 7) for r in self.active])
        self.plans = [e for e in self.events if e['topic'] == '/plan' and e.get('poses')
                      and self.initial['wall'] <= e['wall'] <= self.rows[-1]['wall']]
        if not self.plans:
            raise ValueError('No actual /plan captured; cannot draw an expected path')
        self.grid_cache = {}
        self.focus = min(self.active, key=lambda r: r.get('clearance_m', math.inf))
        coords = np.vstack((self.truth[:, :2], self.target[:2], self.path(self.plans[0])[:, :2]))
        lo, hi = coords.min(axis=0) - 4, coords.max(axis=0) + 4
        self.bounds = (lo[0], hi[0], lo[1], hi[1])

    def offset_for(self, frame):
        if frame.lstrip('/') == 'map':
            return np.zeros(2)
        if frame.lstrip('/') == 'odom':
            return self.offset
        raise ValueError('Unsupported frame (must not guess a transform): ' + frame)

    def path(self, event):
        points = np.asarray(event['poses'], dtype=float).reshape(-1, 3).copy()
        points[:, :2] += self.offset_for(event['frame'])
        return points

    def latest(self, topic, wall):
        return next((e for e in reversed(self.events) if e['topic'] == topic and e['wall'] <= wall), None)

    def grid(self, event):
        if event['file'] not in self.grid_cache:
            with np.load(self.raw / event['file']) as source:
                self.grid_cache[event['file']] = source['data']
        return self.grid_cache[event['file']]

    def draw_grid(self, ax, event, cost=False):
        if event is None:
            return
        if abs(event['origin'][2]) > 1e-6:
            raise ValueError('Rotated map origins require an explicit image transform')
        data = self.grid(event)
        x, y = np.asarray(event['origin'][:2]) + self.offset_for(event['frame'])
        res = event['resolution']
        extent = [x, x + event['width'] * res, y, y + event['height'] * res]
        if cost:
            pixels = np.ma.masked_where(data <= 0, data)
            ax.imshow(pixels, origin='lower', extent=extent, cmap=COST, vmin=0, vmax=100,
                      interpolation='nearest', zorder=1)
        else:
            pixels = np.where(data < 0, 0, np.where(data >= 65, 2, 1))
            ax.imshow(pixels, origin='lower', extent=extent, cmap=WATER, vmin=0, vmax=2,
                      interpolation='nearest', zorder=0)

    def map(self, ax, wall, cost_topic=None):
        self.draw_grid(ax, self.latest('/map', wall))
        if cost_topic:
            self.draw_grid(ax, self.latest(cost_topic, wall), cost=True)

    def hull(self, ax, pose, color=BLUE, alpha=0.12, label=None):
        x, y, heading = pose
        c, s = math.cos(heading), math.sin(heading)
        points = np.asarray(self.scene['footprint']) @ np.array([[c, s], [-s, c]]) + [x, y]
        ax.add_patch(Polygon(points, facecolor=matplotlib.colors.to_rgba(color, alpha),
                             edgecolor=color, linewidth=1.2, label=label, zorder=7))
        ax.arrow(x, y, 1.3 * c, 1.3 * s, color=color, width=0.035,
                 head_width=0.28, length_includes_head=True, zorder=9)

    def target_marker(self, ax, label=True, arrow_length=1.5):
        x, y, heading = self.target
        ax.scatter([x], [y], marker='*', s=190, c=PURPLE, edgecolor='white', linewidth=0.7,
                   label='期望末端位置' if label else None, zorder=10)
        ax.arrow(x, y, arrow_length * math.cos(heading), arrow_length * math.sin(heading),
                 width=arrow_length*.025, head_width=arrow_length*.2, color=PURPLE, zorder=9,
                 length_includes_head=True)

    def obstacle(self, ax, wall, trail=False):
        moves = [e for e in self.trace if e.get('event') == 'move_dynamic_obstacle' and e['wall'] <= wall]
        if moves:
            x, y = moves[-1]['position']
            if [x, y] != [15, 20]:
                ax.add_patch(Rectangle((x - 1, y - 1), 2, 2, fill=False, edgecolor=ORANGE,
                                       linewidth=1.8, linestyle='--', zorder=5,
                                       label='动态障碍真值轮廓'))
        if trail:
            moving = [e['position'] for e in moves if e['position'] != [15, 20]]
            if moving:
                p = np.asarray(moving)
                ax.plot(p[:, 0], p[:, 1], ':', color=ORANGE, linewidth=1.4)


def setup(ax, title, bounds):
    ax.set(title=title, xlabel='东向 X / m', ylabel='北向 Y / m', aspect='equal',
           xlim=bounds[:2], ylim=bounds[2:])
    ax.grid(alpha=0.18, zorder=-1)


def draw_path(ax, data, event, color, label, style='-', width=2, alpha=1):
    if event is not None and event.get('poses'):
        p = data.path(event)
        ax.plot(p[:, 0], p[:, 1], style, color=color, lw=width, label=label, alpha=alpha, zorder=4)


def comparison(data):
    fig, axes = plt.subplots(1, 2, figsize=(13, 6), gridspec_kw={'width_ratios': [1.6, 1]}, layout='constrained')
    ax = axes[0]
    # A moving obstacle's old inflation field must not be mistaken for the
    # costmap in force along the entire later track. Show the static base here;
    # time-matched costmaps are provided in maps.png and the animated replay.
    data.map(ax, data.focus['wall'])
    draw_path(ax, data, data.plans[0], '#7a8797', '首次全局规划', '--', 1.5)
    draw_path(ax, data, data.latest('/plan', data.focus['wall']), GREEN, '避障时全局规划', '-', 2.4)
    removal = next((e for e in data.trace if e.get('event') == 'move_dynamic_obstacle'
                    and e['position'] == [15, 20]), None)
    if removal:
        later = next((e for e in data.plans if e['wall'] > removal['wall']+2), None)
        draw_path(ax, data, later, '#149da9', '障碍移开后重规划', ':', 2.1)
    ax.plot(data.truth[:, 0], data.truth[:, 1], color=BLUE, lw=2.3, label='实际轨迹：VRX 真值', zorder=6)
    ax.plot(data.ekf[:, 0], data.ekf[:, 1], '--', color=ORANGE, lw=1.4, label='飞控 EKF 轨迹', zorder=6)
    ax.scatter(*data.truth[0, :2], c=BLUE, marker='o', s=45, edgecolor='white', zorder=9)
    data.hull(ax, [*data.focus['truth'][:2], data.focus['truth'][3]], label='最近通过时的船体')
    data.obstacle(ax, data.focus['wall'], trail=True)
    data.target_marker(ax)
    setup(ax, '规划路径与实际航迹（同一次运行）', data.bounds)
    ax.legend(loc='upper left', fontsize=8.5, framealpha=0.95)
    ax.text(0.01, 0.01, f"绿色规划与障碍轮廓：t={data.focus['sim']-data.t0:.1f} s；蓝色为全程航迹\n背景为静态地图；动态障碍随后移开，规划持续更新。",
            transform=ax.transAxes, fontsize=8.5, bbox={'facecolor': 'white', 'alpha': .85, 'edgecolor': 'none'})
    ax = axes[1]
    target, actual = data.target, np.asarray(data.result['at_goal'])
    ax.set_facecolor('#f6f9fc')
    ax.plot(data.truth[:, 0], data.truth[:, 1], color=BLUE, lw=2)
    ax.plot(data.ekf[:, 0], data.ekf[:, 1], '--', color=ORANGE, lw=1.3)
    ax.add_patch(Circle(target[:2], 0.5, facecolor='#eee7fb', edgecolor=PURPLE, linestyle='--',
                        alpha=.75, label='位置验收半径 0.5 m'))
    ax.plot([actual[0], target[0]], [actual[1], target[1]], ':', color='#555555', lw=1.3)
    data.target_marker(ax, arrow_length=.8)
    ax.scatter(*actual[:2], marker='o', s=65, c=BLUE, edgecolor='white', label='任务完成瞬间真值', zorder=10)
    ax.arrow(*actual[:2], .8*math.cos(actual[2]), .8*math.sin(actual[2]), color=BLUE,
             width=.02, head_width=.16, length_includes_head=True, zorder=9)
    setup(ax, '末端位置与航向放大', [target[0]-1.2, target[0]+1.8, target[1]-1.2, target[1]+1.8])
    ax.legend(loc='upper left', fontsize=8.5)
    text = (f'期望：({target[0]:.2f}, {target[1]:.2f}) m，{math.degrees(target[2]):.1f}°\n'
            f'实际：({actual[0]:.2f}, {actual[1]:.2f}) m，{math.degrees(actual[2]):.1f}°\n'
            f"位置误差 {data.result['true_position_error_m']:.3f} m\n"
            f"航向误差 {math.degrees(data.result['true_yaw_error_rad']):.2f}°")
    ax.text(.03, .03, text, transform=ax.transAxes, fontsize=10, linespacing=1.7,
            bbox={'facecolor': 'white', 'alpha': .95, 'edgecolor': '#e1e5ea', 'pad': 8})
    fig.suptitle('VRX WAM-V · ArduRover 4.4.0 · Nav2 导航对照', fontsize=16)
    fig.savefig(data.output / 'trajectory_comparison.png', dpi=160)
    plt.close(fig)


def maps(data):
    fig, axes = plt.subplots(1, 3, figsize=(16, 6), layout='constrained')
    wall, t = data.focus['wall'], data.focus['sim'] - data.t0
    ax = axes[0]
    data.map(ax, wall)
    boundary = np.asarray(data.area['navigable_boundary'] + [data.area['navigable_boundary'][0]])
    ax.plot(boundary[:, 0], boundary[:, 1], color='#637383', lw=1)
    ax.plot(data.truth[:, 0], data.truth[:, 1], color=BLUE, lw=1.6)
    data.target_marker(ax, label=False)
    ax.add_patch(Rectangle((data.bounds[0], data.bounds[2]), data.bounds[1]-data.bounds[0],
                           data.bounds[3]-data.bounds[2], fill=False, edgecolor=BLUE, linestyle='--'))
    ax.text(35, 30, '码头禁航区', color='white', ha='center', fontsize=9)
    ax.annotate('已知静态障碍', xy=(10, 0), xytext=(15, -20), fontsize=9,
                arrowprops={'arrowstyle': '->', 'color': '#555555'})
    setup(ax, '静态导航地图 · 100 m × 100 m', [-52, 52, -52, 52])
    ax.legend(handles=[Patch(facecolor='#edf7fc', label='可航水域'),
                       Patch(facecolor='#39424e', label='禁航 / 已知占据'),
                       Patch(facecolor='#d7dce1', label='未知区域')], loc='lower left', fontsize=8)
    for ax, topic, title in [(axes[1], '/global_costmap/costmap', '全局代价地图'),
                             (axes[2], '/local_costmap/costmap', '局部代价地图 + DWB 预测')]:
        data.map(ax, wall, topic)
        draw_path(ax, data, data.latest('/plan', wall), GREEN, '当前全局路径')
        upto = [r for r in data.active if r['wall'] <= wall]
        points = np.asarray([r['truth'] for r in upto])
        ax.plot(points[:, 0], points[:, 1], color=BLUE, lw=2, label='已航行真值')
        data.hull(ax, [*data.focus['truth'][:2], data.focus['truth'][3]])
        data.obstacle(ax, wall)
        data.target_marker(ax, label=False)
        if topic.startswith('/local_'):
            draw_path(ax, data, data.latest('/local_plan', wall), PURPLE, 'DWB 局部预测', '-', 2.7)
        setup(ax, f'{title}\nt = {t:.1f} s', data.bounds)
        ax.legend(loc='upper left', fontsize=8)
    fig.suptitle('地图来自 /map 与 Nav2 实际发布的 costmap；动态障碍由 LiDAR 加入代价地图', fontsize=14)
    fig.text(.5, .005, '浅蓝：可航水域　黄→红：障碍膨胀代价增高　紫红：内切半径代价　深灰：占据　橙色虚框：动态障碍真值',
             ha='center', fontsize=10)
    fig.savefig(data.output / 'maps.png', dpi=160, bbox_inches='tight')
    plt.close(fig)


def replay(data):
    times = np.arange(data.t0, data.rows[-1]['sim'], 1.0)
    times = np.append(times, data.rows[-1]['sim'])
    sim = np.asarray([r['sim'] for r in data.rows])
    fig, axes = plt.subplots(1, 2, figsize=(12.8, 6.4), dpi=100)
    fig.subplots_adjust(left=.06, right=.97, bottom=.15, top=.86, wspace=.22)
    out = data.output / 'navigation_replay.mp4'
    cmd = ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgba',
           '-s', '1280x640', '-r', '6', '-i', '-', '-an', '-c:v', 'libx264', '-crf', '20',
           '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(out)]
    process = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    title = fig.suptitle('', fontsize=15)
    fig.text(.5, .035, '绿：当前全局规划　蓝：实际真值轨迹与船体　橙虚线：EKF　紫：DWB 局部预测 / 目标位姿\n'
             '采样间隔 1 仿真秒，6 倍速回放；到点后保留漂移观测。背景为最近收到的真实代价地图。',
             ha='center', fontsize=10)
    try:
        for t in times:
            index = min(max(int(np.searchsorted(sim, t, side='right'))-1, 0), len(data.rows)-1)
            row = data.rows[index]
            wall = row['wall']
            points = np.asarray([r['truth'] for r in data.rows[:index+1]])
            estimates = np.asarray([r.get('ekf', [np.nan]*7) for r in data.rows[:index+1]])
            for ax, topic, panel in [(axes[0], '/global_costmap/costmap', '航行全景'),
                                      (axes[1], '/local_costmap/costmap', '船体周围 / 局部控制')]:
                ax.clear()
                data.map(ax, wall, topic)
                plan = data.latest('/plan', wall)
                draw_path(ax, data, plan, GREEN, None, width=2.1)
                ax.plot(points[:, 0], points[:, 1], color=BLUE, lw=2.2, zorder=6)
                ax.plot(estimates[:, 0], estimates[:, 1], '--', color=ORANGE, lw=1.2, zorder=6)
                data.target_marker(ax, label=False)
                data.obstacle(ax, wall)
                data.hull(ax, [*row['truth'][:2], row['truth'][3]])
                bounds = data.bounds
                if ax is axes[1]:
                    local = data.latest('/local_plan', wall)
                    # Do not show a stale controller rollout after the action ends.
                    if local and row['phase'] == 'navigation' and row['sim']-local['stamp'] < 1:
                        draw_path(ax, data, local, PURPLE, None, width=2.6)
                    x, y = row['truth'][:2]
                    bounds = [x-7, x+7, y-7, y+7]
                setup(ax, panel, bounds)
                grid = data.latest(topic, wall)
                age = row['sim']-grid['stamp'] if grid else math.nan
                ax.text(.02, .02, f'地图源时间年龄 {age:.2f} s', transform=ax.transAxes, fontsize=8,
                        bbox={'facecolor': 'white', 'alpha': .8, 'edgecolor': 'none'})
            phase = '导航中' if row['phase'] == 'navigation' else '任务结束后的漂移观测'
            title.set_text(f'真实记录回放 · t = {row["sim"]-data.t0:.1f} s · {phase}')
            fig.canvas.draw()
            process.stdin.write(fig.canvas.buffer_rgba())
    finally:
        process.stdin.close()
        result = process.wait(timeout=30)
        plt.close(fig)
    if result:
        raise RuntimeError('ffmpeg video encoding failed')
    subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-i', str(out),
                    '-filter_complex', '[0:v]scale=1024:-1:flags=lanczos,split[a][b];[a]palettegen[p];[b][p]paletteuse',
                    '-loop', '0', str(data.output / 'navigation_replay.gif')], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--label', default='dynamic_navigation')
    parser.add_argument('--animate', action='store_true')
    args = parser.parse_args()
    data = Evidence(args.run_dir, args.label)
    comparison(data)
    maps(data)
    if args.animate:
        replay(data)
    sources = [data.raw/'events.jsonl', data.run/(args.label+'.jsonl'),
               data.run/(args.label+'_result.json'), data.run/'map_alignment.json',
               data.run/'generated/water.yaml', data.run/'generated/water.pgm',
               data.run/'generated/scene.json', *sorted(data.raw.glob('*.npz'))]
    manifest = {'run_dir': str(data.run), 'case': args.label, 'target': data.target.tolist(),
                'actual_at_goal': data.result['at_goal'], 'result': data.result,
                'global_plan_messages': len(data.plans), 'focus_elapsed_sim_s': data.focus['sim']-data.t0,
                'meaning': 'Nav2 paths are spatial references updated online, not a time-parameterized desired vessel trajectory. Actual pose at action completion, not after drift.',
                'synchronization': 'No future observations: each frame uses last message received before the trace sample. Source times are retained.',
                'coordinates': 'map ENU metres; EKF trace already map-aligned; raw odom paths/grids use the run horizontal map offset only.',
                'plotter_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                'sources_sha256': {str(p.relative_to(data.run)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}}
    manifest['artifacts_sha256'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in data.output.iterdir()
                                    if p.suffix in ('.png', '.gif', '.mp4')}
    (data.output/'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False)+'\n')
    print(data.output)


if __name__ == '__main__':
    main()
