#!/usr/bin/env python3
"""README 그림 생성: docs/img/racing_lines.png (트랙 맵 + 속도별 레이싱라인), docs/img/auto_mapping.png (GT 없는 매핑 결과)."""
import os, sys, numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap
HERE = os.path.dirname(os.path.abspath(__file__)); WS = os.path.dirname(HERE); os.chdir(WS)
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11})
BG = '#0d1117'; FG = '#e6edf3'; MUTED = '#8b949e'
WALLS = ListedColormap([(0, 0, 0, 0), (0.55, 0.58, 0.62, 1.0)])

def draw_map(ax, mapf):
    z = np.load(mapf); occ = z['occ']; ox, oy = z['origin']; R = float(z['res']); H, W = occ.shape
    ax.imshow(occ.astype(float), origin='lower', extent=[ox, ox + W * R, oy, oy + H * R], cmap=WALLS, interpolation='nearest')
    return z

def draw_line(ax, linef, lw=3.4):
    D = np.loadtxt(linef, delimiter=','); P = np.vstack([D[:, 1:3], D[:1, 1:3]]); v = D[:, 5]
    lc = LineCollection(np.stack([P[:-1], P[1:]], 1), cmap='plasma', linewidth=lw, capstyle='round'); lc.set_array(v); lc.set_clim(2, 11)
    ax.add_collection(lc); ax.plot(*D[0, 1:3], 'o', ms=10, mfc='#3fb950', mec='white', mew=1.6, zorder=5)
    return lc, D

def racing_lines():
    fig = plt.figure(figsize=(12, 7.2), facecolor=BG)
    specs = [('maps/iros26_map.npz', 'maps/iros26_combo.csv', 'IROS 2026 · Final track', 'race-10  78.9 s   ·   0 race collisions', [0.03, 0.06, 0.42, 0.86]),
             ('maps/porto_map.npz', 'maps/porto_mt_best.csv', 'Porto · Qualifying track', 'flying lap  5.66 s   ·   10 laps, 0 collisions', [0.47, 0.22, 0.36, 0.66])]
    for mapf, linef, title, sub, rect in specs:
        ax = fig.add_axes(rect, facecolor=BG); draw_map(ax, mapf); lc, D = draw_line(ax, linef)
        ax.set_aspect('equal'); ax.axis('off'); ax.autoscale_view()
        ax.set_title(title, color=FG, fontsize=15, fontweight='bold', pad=6)
        ax.text(0.5, -0.03, sub, transform=ax.transAxes, ha='center', va='top', color=MUTED, fontsize=11)
        ax.text(0.5, -0.09, f'top speed {D[:, 5].max():.1f} m/s', transform=ax.transAxes, ha='center', va='top', color=MUTED, fontsize=10)
    cax = fig.add_axes([0.89, 0.22, 0.016, 0.66]); cb = fig.colorbar(lc, cax=cax)
    cb.set_label('planned speed  [m/s]', color=FG); cb.ax.tick_params(colors=FG); cb.outline.set_edgecolor(MUTED)
    fig.text(0.65, 0.06, 'minimum-time lines · curvature-dependent grip · speed-linked wall margin   (green = start)', ha='center', color=MUTED, fontsize=10)
    fig.savefig('docs/img/racing_lines.png', dpi=100, facecolor=BG); plt.close(fig)

def auto_mapping(rec_dir):
    """rec_dir: race/test_explore_offline.py 출력 폴더(auto_map.npz, auto_line.csv, auto_rec.npz)"""
    fig = plt.figure(figsize=(12, 7.2), facecolor=BG)
    ax1 = fig.add_axes([0.03, 0.08, 0.30, 0.82], facecolor=BG); ax2 = fig.add_axes([0.36, 0.08, 0.30, 0.82], facecolor=BG)
    rec = np.load(os.path.join(rec_dir, 'auto_rec.npz'))
    draw_map(ax1, os.path.join(rec_dir, 'auto_map.npz')); le = int(rec['lap_end'])
    ax1.plot(rec['x'][:le], rec['y'][:le], color='#58a6ff', lw=1.6, label='SLAM trajectory')
    ax1.plot(rec['x'][0], rec['y'][0], 'o', ms=10, mfc='#3fb950', mec='white', mew=1.6)
    C = np.loadtxt(os.path.join(rec_dir, 'auto_center.csv'), delimiter=','); ax1.plot(C[:, 0], C[:, 1], '--', color='#e3b341', lw=1.2, label='extracted centerline')
    leg = ax1.legend(loc='lower center', bbox_to_anchor=(0.5, -0.08), frameon=False, fontsize=9, labelcolor=FG)
    ax1.set_title('1 · Map one lap (no GT)', color=FG, fontsize=13, fontweight='bold')
    draw_map(ax2, os.path.join(rec_dir, 'auto_map.npz')); lc, _ = draw_line(ax2, os.path.join(rec_dir, 'auto_line.csv'), lw=3.0)
    ax2.set_title('2 · Auto min-time line', color=FG, fontsize=13, fontweight='bold')
    for a in (ax1, ax2): a.set_aspect('equal'); a.axis('off'); a.autoscale_view()
    tx = fig.add_axes([0.69, 0.08, 0.29, 0.82], facecolor=BG); tx.axis('off')
    lines = [('Unknown-track pipeline', FG, 14, 'bold'), ('', FG, 6, 'normal'),
             ('explore.py', '#58a6ff', 11, 'bold'), ('follow-the-gap + online SLAM', MUTED, 10, 'normal'),
             ('IMU yaw · encoders · LiDAR scan-to-map', MUTED, 10, 'normal'), ('', FG, 6, 'normal'),
             ('auto_track.py  stage A  (~5 s)', '#e3b341', 11, 'bold'), ('occupancy map · centerline · min-curvature', MUTED, 10, 'normal'), ('', FG, 6, 'normal'),
             ('handover → localizer + MPC', '#3fb950', 11, 'bold'), ('', FG, 6, 'normal'),
             ('auto_track.py  stage B', '#f778ba', 11, 'bold'), ('min-time line, hot-swapped while racing', MUTED, 10, 'normal'), ('', FG, 6, 'normal'),
             ('synthetic IROS26 test:', FG, 10, 'bold'), ('centerline error  4–6 cm (mean)', MUTED, 10, 'normal'), ('est. lap  7.9 s  vs hand-tuned 7.67 s', MUTED, 10, 'normal')]
    y = 0.95
    for t, c, fs, fw in lines:
        tx.text(0.0, y, t, color=c, fontsize=fs, fontweight=fw, va='top', transform=tx.transAxes); y -= 0.022 if fs <= 6 else 0.05
    fig.savefig('docs/img/auto_mapping.png', dpi=100, facecolor=BG); plt.close(fig)

if __name__ == '__main__':
    racing_lines()
    if len(sys.argv) > 1: auto_mapping(sys.argv[1])
    print('saved docs/img/*.png')
