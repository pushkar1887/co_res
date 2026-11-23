"""Figures 3.1 (system architecture) and 5.1 (ESP32 Wokwi fixture) for the B.Tech report.
Run from the repo root: python report/make_figures.py. Fonts are sized for a 6-inch-wide figure in the report."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['mathtext.fontset'] = 'stix'
NAVY, BLUE, GREY = '#1B365D', '#2563EB', '#475569'
FT, FS, FL = 13, 10.5, 10            # box title, box subtitle, arrow label (pt in a 10-inch figure; x0.6 in the report)
NL = chr(10)

# ---------------------------------------------------------------- Figure 3.1: architecture
fig, ax = plt.subplots(figsize=(10.6, 4.9), dpi=300)
ax.set_xlim(0, 10.6); ax.set_ylim(0, 4.9); ax.axis('off')


def box(x, y, w, h, title, sub, fc='#F1F5F9', ours=False):
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h, boxstyle='round,pad=0.02,rounding_size=0.08',
                                ec=BLUE if ours else NAVY, fc='#DBEAFE' if ours else fc, lw=1.8 if ours else 1.2))
    lines = sub.count(NL) + 1
    top = y + (0.32 + 0.21 * lines) / 2
    ax.text(x, top - 0.13, title, ha='center', va='center', fontsize=FT, weight='bold', color=BLUE if ours else NAVY)
    ax.text(x, top - 0.31, sub, ha='center', va='top', fontsize=FS, color=GREY, linespacing=1.2)


def arrow(p, q, txt=None, color=GREY, ls='-', tx=0.0, ty=0.1, ha='center'):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle='-|>', mutation_scale=12, lw=1.2, color=color, ls=ls, shrinkA=1, shrinkB=1))
    if txt:
        ax.text((p[0] + q[0]) / 2 + tx, (p[1] + q[1]) / 2 + ty, txt, ha=ha, fontsize=FL, color=color, style='italic')


W, H = 2.1, 0.9
X1, X2, X3, X4 = 1.1, 3.85, 6.6, 9.35
Y1, Y2, Y3 = 4.15, 2.7, 1.2
h = W / 2
box(X1, Y1, W, H, 'Overhead camera', r'10 Hz, $\sigma$ = 10 mm / 1$\degree$')
box(X1, 1.95, W, 1.3, 'Onboard sensors', 'gyro: ARW, bias, fault' + NL + 'encoders: 0.576 mm/count' + NL + 'both at 50 Hz')
box(X2, Y1, W, H, 'Loss channel', 'blackout 0.5–4 s at a turn')
box(X2, Y2, W, H, r'EKF  $[x,\ y,\ \theta,\ b_g]$', 'predict: encoders + gyro' + NL + 'update: camera')
box(X2, Y3, W, H, 'Consistency monitor', 'gyro vs encoder heading' + NL + r'alarm if $|D| > 0.6\degree$', ours=True)
box(X3, Y3, W, H, 'Speed policy', 'N / SB / SA / SO' + NL + r'cap $\alpha v$ = 0.15 m/s', ours=True)
box(X3, Y2, W, H, 'Pure Pursuit', r'10 Hz, $L_d$ = 0.30 m' + NL + r'$\omega_c = 2 v\, y_g / d^2$')
box(X4, Y2, W, H, 'Motors + robot', 'diff-drive, b = 0.20 m' + NL + '0.5 m/s, lag 0.12 s')
box(X4, Y1, W, H, 'Scoring (truth)', 'cross-track error e(t)' + NL + '±30 mm lane', fc='#FFFFFF')

GX = (X2 + h + X3 - h) / 2                                   # x of the vertical blackout-flag line, mid-gap
arrow((X1 + h, Y1), (X2 - h, Y1), 'pose', ty=0.13)
arrow((X2, Y1 - 0.45), (X2, Y2 + 0.45), 'delivered /' + NL + 'missing', tx=-0.08, ty=-0.12, ha='right')
arrow((X1 + h, 2.3), (X2 - h, 2.6))
arrow((X1 + h, 1.6), (X2 - h, 1.2), color=BLUE)
arrow((X2 + h, Y3), (X3 - h, Y3), 'alarm', color=BLUE, ty=-0.33)
arrow((X2 + h, Y2), (X3 - h, Y2), r'$\hat{x}$', ty=0.12, tx=-0.17)
ax.plot([X2 + h, GX, GX], [4.25, 4.25, 1.45], color=GREY, lw=1.2)          # blackout flag, routed down the gap
arrow((GX, 1.45), (X3 - h, 1.45))
ax.text(GX + 0.08, 3.75, 'blackout flag', fontsize=FL, color=GREY, style='italic', va='center')
arrow((X3, Y3 + 0.45), (X3, Y2 - 0.45), r'$v_c$', tx=0.2, ty=-0.05, color=BLUE)
arrow((X3 + h, Y2), (X4 - h, Y2), r'$u_L,u_R$', ty=0.12)
arrow((X4, Y2 + 0.45), (X4, Y1 - 0.45), ls='--')
FX = X4 + 0.75                                               # feedback path leaves the robot box near its right edge
ax.plot([FX, FX, X1], [Y2 - 0.45, 0.42, 0.42], color=GREY, lw=1.0, ls=':')
arrow((X1, 0.42), (X1, 1.3), ls=':')
ax.text(5.2, 0.17, 'true motion drives the sensors (injected faults: gyro bias step, wheel slip, wheel-radius error)',
        ha='center', fontsize=FL, color=GREY, style='italic')
ax.text(X4 - 0.25, Y3, 'blue = proposed' + NL + '(monitor + SA)', ha='center', va='center', fontsize=FL, color=BLUE)
fig.savefig('report/fig_architecture.png', bbox_inches='tight', facecolor='white')

# ---------------------------------------------------------------- Figure 5.1: ESP32 replay fixture (from diagram.json)
fig, ax = plt.subplots(figsize=(10, 4.4), dpi=300)
ax.set_xlim(0, 10); ax.set_ylim(0, 4.4); ax.axis('off')
ax.add_patch(FancyBboxPatch((0.25, 0.5), 2.75, 3.65, boxstyle='round,pad=0.02,rounding_size=0.1', ec=NAVY, fc='#F1F5F9', lw=1.4))
ax.text(1.625, 3.85, 'ESP32 DevKitC-V4', ha='center', fontsize=FT, weight='bold', color=NAVY)
ax.text(1.625, 3.5, 'float32 firmware (estimator.h)', ha='center', fontsize=FS, color=GREY)
for i, t in enumerate(['EKF + monitor + Pure Pursuit', 'replays trace_healthy.h and', 'trace_gyro_fault.h from flash',
                       '(706 steps of 20 ms each)', 'checks every step against the', 'browser reference (V8)']):
    ax.text(0.4, 3.08 - 0.3 * i, t, fontsize=FS, color=GREY)
GNDX = 9.7
rows = [('GPIO25', 'L wheel', '#16A34A', 'left-wheel command (PWM)'), ('GPIO26', 'R wheel', '#16A34A', 'right-wheel command (PWM)'),
        ('GPIO27', 'camera', '#2563EB', 'on = camera fix fresh'), ('GPIO14', 'alarm', '#DC2626', 'monitor alarm (SA slows)')]
for i, (pin, name, col, role) in enumerate(rows):
    y = 3.7 - 0.66 * i
    ax.text(3.07, y + 0.08, pin, ha='left', va='bottom', fontsize=FS, weight='bold', color=NAVY)
    ax.plot([3.0, 4.4], [y, y], color=col, lw=1.8)
    ax.add_patch(FancyBboxPatch((4.4, y - 0.1), 0.6, 0.2, boxstyle='round,pad=0.0,rounding_size=0.03', ec=GREY, fc='#FDE68A', lw=1))
    ax.text(4.7, y + 0.18, '220 Ω', ha='center', fontsize=9, color=GREY)
    ax.plot([5.0, 5.6], [y, y], color=col, lw=1.8)
    ax.add_patch(plt.Circle((5.76, y), 0.15, color=col))
    ax.text(6.0, y, f'LED "{name}": {role}', va='center', fontsize=FS, color=NAVY)
    ax.plot([5.76, 5.76, GNDX], [y - 0.15, y - 0.32, y - 0.32], color='#111827', lw=0.8)
yb = 0.9
ax.text(3.07, yb + 0.08, 'GPIO13', ha='left', va='bottom', fontsize=FS, weight='bold', color=NAVY)
ax.plot([3.0, 5.5], [yb, yb], color='#EA580C', lw=1.8)
ax.add_patch(FancyBboxPatch((5.5, yb - 0.17), 0.34, 0.34, boxstyle='round,pad=0.0,rounding_size=0.04', ec=GREY, fc='#F8FAFC', lw=1))
ax.add_patch(plt.Circle((5.67, yb), 0.11, color='#DC2626'))
ax.text(6.0, yb + 0.14, 'push button (input pull-up): injects', va='center', fontsize=FS, color=NAVY)
ax.text(6.0, yb - 0.16, '+1°/s into the replayed gyro', va='center', fontsize=FS, color=NAVY)
ax.plot([5.84, 5.92, 5.92, GNDX], [yb - 0.1, yb - 0.1, yb - 0.38, yb - 0.38], color='#111827', lw=0.8)
ax.plot([GNDX, GNDX], [yb - 0.38, 3.38], color='#111827', lw=1.4)
ax.text(GNDX + 0.07, 2.0, 'GND', fontsize=9.5, color='#111827', rotation=90, va='center')
ax.text(1.625, 0.15, 'TX / RX → serial monitor (CSV every 0.1 s, parity summary)', ha='center', fontsize=FL, color=GREY, style='italic')
fig.savefig('report/fig_wokwi.png', bbox_inches='tight', facecolor='white')
print('ok')
