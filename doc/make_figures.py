#!/usr/bin/env python3
"""
Draw the README figures that explain the challenge rules (host only, needs matplotlib):

    doc/legend.png      what the markers in Stage / RViz mean, collisions, goal order
    doc/goal-check.png  when a goal counts as reached (position AND heading)
    doc/hard-grid.png   the template's planning grid on the HARD map with a BFS path (seed 42)

    python3 doc/make_figures.py

The numbers come from the challenge config / scorer, so the drawings are to scale.
"""

import math
import sys
from collections import deque
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import patches  # noqa: E402
from matplotlib.transforms import Affine2D  # noqa: E402

OUT = Path(__file__).resolve().parent
PKG = OUT.parent / 'challenge' / 'src' / 'coderun_challenge'
sys.path.insert(0, str(PKG))
from coderun_challenge import sampler  # noqa: E402

CELL, CLEARANCE = 0.5, 0.35    # coderun_student/python/solution.py build_grid()
GOAL_R = 0.3            # config/*_goals.yaml goal_radius (m)
YAW_TOL = 20.0          # config/*_goals.yaml yaw_tolerance_deg
COLLIDE = 0.28          # scorer.py COLLIDE_DIST: robot centre to wall (m)
RECT = (0.5, 0.36)      # goal marker footprint (goal_server.py RECT)
ROBOT = (0.44, 0.38)    # pioneer2dx size (m)

CYAN, GREEN, RED, WALL = '#19cce6', '#33e633', '#e03030', '#4d4d4d'
OK, BAD = '#1a9e3a', '#c62828'


def pose_marker(ax, x, y, yaw_deg, color, label, dots=0):
    """Goal / start marker as drawn in Stage + RViz: rectangle, heading arrow, ring, number."""
    t = Affine2D().rotate_deg_around(x, y, yaw_deg) + ax.transData
    ax.add_patch(patches.Rectangle((x - RECT[0] / 2, y - RECT[1] / 2), *RECT, color=color,
                                   alpha=0.75, transform=t, zorder=2))
    a = math.radians(yaw_deg)
    ax.annotate('', xy=(x + 0.27 * math.cos(a), y + 0.27 * math.sin(a)),
                xytext=(x - 0.2 * math.cos(a), y - 0.2 * math.sin(a)),
                arrowprops=dict(arrowstyle='-|>', color='white', lw=2.2), zorder=3)
    ax.add_patch(patches.Circle((x, y), GOAL_R, fill=False, ec=color, lw=1.5, zorder=2))
    ax.text(x, y + 0.42, label, ha='center', va='bottom', fontsize=11, weight='bold', zorder=4)
    for k in range(dots):
        ax.add_patch(patches.Circle((x + (k - (dots - 1) / 2) * 0.11 - 0.0, y + 0.36), 0.03,
                                    color='white', ec='gray', lw=0.6, zorder=3))


def robot(ax, x, y, yaw_deg, alpha=1.0):
    t = Affine2D().rotate_deg_around(x, y, yaw_deg) + ax.transData
    ax.add_patch(patches.Rectangle((x - ROBOT[0] / 2, y - ROBOT[1] / 2), *ROBOT, color=RED,
                                   alpha=alpha, transform=t, zorder=5))
    a = math.radians(yaw_deg)
    ax.plot([x, x + 0.24 * math.cos(a)], [y, y + 0.24 * math.sin(a)], color='navy', lw=2, zorder=6)
    ax.plot(x, y, 'k.', ms=4, zorder=6)


def note(ax, xy, text, xytext, color='black'):
    ax.annotate(text, xy=xy, xytext=xytext, fontsize=9, color=color, zorder=7,
                arrowprops=dict(arrowstyle='->', color=color, lw=1),
                bbox=dict(boxstyle='round,pad=0.3', fc='white', ec=color, lw=0.8))


def setup(ax, xlim, ylim, title):
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect('equal')
    ax.set_facecolor('#f4f4fb')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(title, fontsize=12, weight='bold')


def legend():
    fig, ax = plt.subplots(figsize=(11, 6.2))
    setup(ax, (-0.6, 7.4), (-0.9, 3.6), 'What you see in Stage and RViz')

    # walls + collision band
    ax.add_patch(patches.Rectangle((4.9, -0.9), 0.2, 3.2, color=WALL, zorder=1))
    ax.add_patch(patches.Rectangle((4.9 - COLLIDE, -0.9), COLLIDE, 3.2, color=BAD, alpha=0.18, zorder=1))
    ax.add_patch(patches.Rectangle((5.1, -0.9), COLLIDE, 3.2, color=BAD, alpha=0.18, zorder=1))
    note(ax, (5.0, 1.9), 'wall (gray): plan a path around it', (5.3, 2.9))
    note(ax, (4.75, 0.2), f'collision zone: robot centre closer\nthan {COLLIDE} m to a wall = 1 collision\n'
                          '(-5 EASY / -10 HARD points each)', (5.45, -0.55), BAD)

    # start + robot
    pose_marker(ax, 0.3, 0.0, 0, GREEN, 'S')
    robot(ax, 0.3, 0.0, 0)
    note(ax, (0.3, -0.2), 'S = start (green), robot at (0, 0, 0)\n/odom, /goals and /map share this frame',
         (-0.45, -0.85))
    # laser fan
    ax.add_patch(patches.Wedge((0.3, 0.0), 1.3, -135, 135, color='blue', alpha=0.06, zorder=1))
    note(ax, (1.35, 0.6), '/base_scan: 270°, 5 m', (1.15, 1.05), 'navy')

    # goals
    pose_marker(ax, 2.6, 0.0, 90, CYAN, '1', dots=1)
    pose_marker(ax, 3.6, 2.2, 0, CYAN, '2', dots=2)
    pose_marker(ax, 6.4, 1.0, 225, CYAN, '3', dots=3)
    note(ax, (2.6 + GOAL_R * 0.7, -GOAL_R * 0.7), f'ring = position tolerance\n(radius {GOAL_R} m)', (3.1, -0.75))
    note(ax, (2.6, 0.24), 'white arrow = required heading\n'
                          f'(±{YAW_TOL:.0f}°)', (0.95, 2.6))
    note(ax, (3.6, 2.85), 'number (RViz) / dots (Stage)\n= goal order', (1.2, 3.25))

    # order
    for a, b in (((0.6, 0.0), (2.3, 0.0)), ((2.75, 0.3), (3.3, 2.0)), ((3.95, 2.3), (5.0, 2.9)), ((5.0, 2.9), (6.2, 1.25))):
        ax.annotate('', xy=b, xytext=a, arrowprops=dict(arrowstyle='->', color='gray', lw=1.2,
                                                        ls='--'), zorder=1)
    ax.text(-0.5, 1.55, 'visit goals IN ORDER\n1 → 2 → 3 …  (reaching goal 3\nbefore goal 2 does not count)',
            fontsize=9, color='dimgray', ha='left', zorder=7)
    fig.tight_layout()
    fig.savefig(OUT / 'legend.png', dpi=110)
    plt.close(fig)


def goal_check():
    cases = [
        ('REACHED', OK, (0.12, -0.08), 10, f'inside the ring\nheading 10° off (< {YAW_TOL:.0f}°)'),
        ('NOT reached', BAD, (0.05, 0.1), 60, 'inside the ring\nbut heading 60° off'),
        ('NOT reached', BAD, (0.45, 0.0), 0, f'heading OK\nbut {0.45:.2f} m away (> {GOAL_R} m)'),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(11, 4))
    for ax, (verdict, color, (dx, dy), off, text) in zip(axes, cases):
        setup(ax, (-0.8, 1.0), (-0.75, 0.85), '')
        ax.set_title(verdict, color=color, fontsize=14, weight='bold')
        pose_marker(ax, 0, 0, 90, CYAN, 'goal (yaw 90°)')
        for s in (-1, 1):   # heading tolerance cone
            a = math.radians(90 + s * YAW_TOL)
            ax.plot([0, 0.6 * math.cos(a)], [0, 0.6 * math.sin(a)], color=CYAN, lw=1, ls=':')
        robot(ax, dx, dy, 90 + off, alpha=0.55)
        ax.text(0.1, -0.68, text, ha='center', fontsize=10)
    fig.suptitle(f'A goal counts when the robot is within {GOAL_R} m AND within ±{YAW_TOL:.0f}° '
                 'of the goal heading (both at the same moment)', fontsize=11)
    fig.tight_layout()
    fig.savefig(OUT / 'goal-check.png', dpi=110)
    plt.close(fig)


def hard_grid(seed=42):
    cfg = yaml.safe_load((PKG / 'config' / 'hard_goals.yaml').read_text())
    bitmap = sampler.bitmap_path(PKG, 'hard')
    occ = np.array(Image.open(bitmap).convert('L')) < 128
    h, w = occ.shape[0] // 2 * 2, occ.shape[1] // 2 * 2
    occ = np.flipud(occ[:h, :w].reshape(h // 2, 2, w // 2, 2).max(axis=(1, 3)))    # = /map
    res, size = 0.04, cfg['world_size']
    x0, y0 = cfg['map_origin']['x'] - size / 2, cfg['map_origin']['y'] - size / 2
    to_wall = ndimage.distance_transform_edt(~occ) * res
    n = int(size / CELL)
    grid = [[to_wall[int((r + .5) * CELL / res), int((c + .5) * CELL / res)] >= CLEARANCE for c in range(n)]
            for r in range(n)]
    goals = sampler.sample_goals(cfg, bitmap, seed)

    def cell(x, y):
        return int((y - y0) // CELL), int((x - x0) // CELL)

    def centre(r, c):
        return x0 + (c + .5) * CELL, y0 + (r + .5) * CELL

    def bfs(a, b):
        prev, q = {a: None}, deque([a])
        while q:
            u = q.popleft()
            if u == b:
                break
            for v in ((u[0] + 1, u[1]), (u[0] - 1, u[1]), (u[0], u[1] + 1), (u[0], u[1] - 1)):
                if 0 <= v[0] < n and 0 <= v[1] < n and grid[v[0]][v[1]] and v not in prev:
                    prev[v] = u
                    q.append(v)
        path = []
        while b is not None:
            path.append(b)
            b = prev[b]
        return path[::-1]

    fig, ax = plt.subplots(figsize=(7.6, 7.6))
    ax.imshow(np.where(occ, 0.3, 1.0), cmap='gray', vmin=0, vmax=1, origin='lower',
              extent=(x0, x0 + size, y0, y0 + size), zorder=0)
    for r in range(n):
        for c in range(n):
            if not grid[r][c]:
                ax.add_patch(patches.Rectangle((x0 + c * CELL, y0 + r * CELL), CELL, CELL, color=BAD,
                                               alpha=0.22, lw=0, zorder=1))
    for k in range(n + 1):
        ax.plot([x0 + k * CELL] * 2, [y0, y0 + size], color='gray', lw=0.3, zorder=1)
        ax.plot([x0, x0 + size], [y0 + k * CELL] * 2, color='gray', lw=0.3, zorder=1)
    pts = [(0.0, 0.0)] + [(g['x'], g['y']) for g in goals]
    for a, b in zip(pts, pts[1:]):
        path = [a] + [centre(*c) for c in bfs(cell(*a), cell(*b))] + [b]
        ax.plot(*zip(*path), color='#1565c0', lw=2, zorder=3)
        ax.plot(*zip(*path[1:-1]), 'o', color='#1565c0', ms=2.5, zorder=3)
    pose_marker(ax, 0, 0, 0, GREEN, 'S')
    robot(ax, 0, 0, 0)
    for i, g in enumerate(goals, start=1):
        pose_marker(ax, g['x'], g['y'], g['yaw_deg'], CYAN, str(i))
    ax.set_xlim(x0, x0 + size)
    ax.set_ylim(y0, y0 + size)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(f'HARD: self.grid ({n}x{n} cells of {CELL} m). Red = blocked (closer than {CLEARANCE} m '
                 'to a wall)\nBlue = BFS path through the free cell centres, goal to goal (seed 42)', fontsize=10)
    fig.tight_layout()
    fig.savefig(OUT / 'hard-grid.png', dpi=100)
    plt.close(fig)


if __name__ == '__main__':
    legend()
    goal_check()
    hard_grid()
    print('wrote', OUT / 'legend.png', OUT / 'goal-check.png', OUT / 'hard-grid.png')
