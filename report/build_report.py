"""Build the B.Tech report from the department template.

Run from the repo root:  python report/build_report.py [path/to/template.docx]
Output: report/BTech_Report.docx and report/BTech_Report.pdf (needs Microsoft Word, via pywin32).

Pages 1-3 (cover, declaration, certificate), the acknowledgement and Appendix A are left exactly as in
the template. The abstract, front-matter lists, Chapters 1-7 and references are generated here, using
paragraph formatting cloned from the template itself (Times New Roman, 12 pt body, 1.5 spacing).
"""
import os, re, shutil, sys, zipfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEMPLATE = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(r'~\Downloads\Btech Report Template 1.docx')
OUT_DOCX = os.path.join(ROOT, 'report', 'BTech_Report.docx')
OUT_PDF = os.path.join(ROOT, 'report', 'BTech_Report.pdf')
FIG = lambda name: os.path.join(ROOT, 'figures', name)
RFIG = lambda name: os.path.join(ROOT, 'report', name)
NAVY, SLATE = '1B365D', '334E68'

# --------------------------------------------------------------------------- XML helpers
esc = lambda s: s.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')


def runs(text, sz=None, bold=False, color=None):
    """Inline markup: **bold**, *italic*, _{sub}, ^{sup}."""
    out = []
    for tok in re.split(r'(\*\*.+?\*\*|\*[^*\s][^*]*?\*|_\{[^}]*\}|\^\{[^}]*\})', text):
        if not tok:
            continue
        b, i, va, t = bold, False, None, tok
        if tok.startswith('**') and tok.endswith('**') and len(tok) > 4:
            b, t = True, tok[2:-2]
        elif tok.startswith('*') and tok.endswith('*') and len(tok) > 2:
            i, t = True, tok[1:-1]
        elif tok.startswith('_{'):
            va, t = 'subscript', tok[2:-1]
        elif tok.startswith('^{'):
            va, t = 'superscript', tok[2:-1]
        rpr = ('<w:b/>' if b else '') + ('<w:i/>' if i else '') + (f'<w:color w:val="{color}"/>' if color else '') + \
              (f'<w:sz w:val="{sz}"/><w:szCs w:val="{sz}"/>' if sz else '') + (f'<w:vertAlign w:val="{va}"/>' if va else '')
        out.append(f'<w:r>{"<w:rPr>" + rpr + "</w:rPr>" if rpr else ""}<w:t xml:space="preserve">{esc(t)}</w:t></w:r>')
    return ''.join(out)


def para(ppr, content):
    return f'<w:p><w:pPr>{ppr}</w:pPr>{content}</w:p>'


B = []                                                   # generated blocks for the main body
P = lambda t: B.append(para('<w:spacing w:after="160"/><w:jc w:val="both"/>', runs(t)))
BUL = lambda t: B.append(para('<w:pStyle w:val="ListBullet"/><w:spacing w:after="80"/><w:jc w:val="both"/>', runs(t)))
PAGE = '<w:p><w:r><w:br w:type="page"/></w:r></w:p>'


def CH(n, title, first=False):
    brk = '' if first else '<w:pageBreakBefore/>'            # break attached to the heading: can never leave an empty page
    B.append(para(f'<w:keepNext/>{brk}<w:spacing w:before="120" w:after="40"/>', runs(f'CHAPTER {n}', bold=True, color=SLATE)))
    B.append(para(f'<w:keepNext/><w:pBdr><w:bottom w:val="single" w:sz="12" w:space="4" w:color="{NAVY}"/></w:pBdr>'
                  '<w:spacing w:after="280"/><w:outlineLvl w:val="0"/>', runs(title, sz=32, bold=True, color=NAVY)))


def SEC(num, title):
    B.append(para('<w:keepNext/><w:spacing w:before="240"/><w:outlineLvl w:val="1"/>', runs(f'{num}  {title}', sz=26, bold=True, color=NAVY)))


def FIGURE(path, width_in, caption):
    B.append(para('<w:keepNext/><w:spacing w:before="160" w:after="0" w:line="240" w:lineRule="auto"/><w:jc w:val="center"/>',
                  runs(f'[[IMG|{path}|{width_in}]]')))
    B.append(para('<w:pStyle w:val="FigCap"/><w:spacing w:before="80" w:after="240" w:line="276" w:lineRule="auto"/><w:jc w:val="center"/>',
                  runs(caption, sz=21, bold=True)))


def EQ(latex, num):
    """Display equation, centred, numbered on the right. `latex` is built up by Word (its equation input is LaTeX here)."""
    raw = f'<w:r><w:t xml:space="preserve">{esc("[[EQ:" + latex + "]]")}</w:t></w:r>'   # raw run: no inline markup parsing
    B.append(para('<w:tabs><w:tab w:val="center" w:pos="4334"/><w:tab w:val="right" w:pos="8669"/></w:tabs>'
                  '<w:spacing w:before="80" w:after="160"/>',
                  f'<w:r><w:tab/></w:r>{raw}<w:r><w:tab/></w:r>{runs(f"({num})")}'))


def TABLE(caption, header, rows, widths):
    assert abs(sum(widths) - 8669) < 400, (caption, sum(widths))
    B.append(para('<w:pStyle w:val="TabCap"/><w:keepNext/><w:spacing w:before="160" w:after="80" w:line="276" w:lineRule="auto"/><w:jc w:val="center"/>',
                  runs(caption, sz=21, bold=True)))
    bd = ''.join(f'<w:{s} w:val="single" w:sz="4" w:space="0" w:color="D0D7DE"/>' for s in ['top', 'left', 'bottom', 'right', 'insideH', 'insideV'])
    x = [f'<w:tbl><w:tblPr><w:tblW w:w="{sum(widths)}" w:type="dxa"/><w:jc w:val="center"/><w:tblBorders>{bd}</w:tblBorders>'
         '<w:tblLayout w:type="fixed"/><w:tblLook w:val="04A0" w:firstRow="1" w:lastRow="0" w:firstColumn="1" w:lastColumn="0" w:noHBand="0" w:noVBand="1"/></w:tblPr>',
         '<w:tblGrid>' + ''.join(f'<w:gridCol w:w="{w}"/>' for w in widths) + '</w:tblGrid>']

    def row(cells, head, shade):
        r = ['<w:tr><w:trPr><w:cantSplit/>' + ('<w:tblHeader/>' if head else '') + '<w:jc w:val="center"/></w:trPr>']
        for c, w in zip(cells, widths):
            fill = NAVY if head else ('F5F7FA' if shade else 'FFFFFF')
            r.append(f'<w:tc><w:tcPr><w:tcW w:w="{w}" w:type="dxa"/><w:shd w:val="clear" w:color="auto" w:fill="{fill}"/>'
                     '<w:tcMar><w:top w:w="60" w:type="dxa"/><w:left w:w="80" w:type="dxa"/><w:bottom w:w="60" w:type="dxa"/><w:right w:w="80" w:type="dxa"/></w:tcMar>'
                     '<w:vAlign w:val="center"/></w:tcPr>')
            r.append(para('<w:spacing w:after="0" w:line="252" w:lineRule="auto"/>', runs(c, sz=20, bold=head, color='FFFFFF' if head else None)))
            r.append('</w:tc>')
        return ''.join(r) + '</w:tr>'
    x.append(row(header, True, False))
    for k, rw in enumerate(rows):
        x.append(row(rw, False, k % 2 == 1))
    B.append(''.join(x) + '</w:tbl>')
    B.append(para('<w:spacing w:after="80"/>', ''))


# =========================================================================== CONTENT
ABSTRACT = [
    ('Background & Motivation: ', 'Warehouse robots are often localized by an overhead camera, and racks hide them from it for seconds at a time, usually at turns. During such a blackout the robot dead-reckons with wheel encoders and a gyro. If one of these sensors fails during the blackout, no external measurement can reveal the fault, and the robot can leave its lane. The usual safeguard is to slow down on every blackout: safe, but it costs time on every healthy one.'),
    ('Methodology: ', 'We propose consistency-gated slowdown. The gyro and the wheel encoders measure the same quantity, the robot’s turn rate. A monitor integrates their disagreement over a 2 s window, and the robot slows to 30% speed only when the disagreement exceeds 0.6°. We model a differential-drive robot with Pure Pursuit steering, an extended Kalman filter, quantized encoders and a biased, noisy MEMS gyro. Before running experiments, we derive closed-form drift envelopes, the detection delay, the regret of delayed slowing and a design window for the threshold. We test pre-registered hypotheses in about 11,000 paired simulation runs and port the estimator and monitor to an ESP32 microcontroller.'),
    ('Results & Analysis: ', 'Under a +1°/s gyro fault during a 4 s blackout at a turn, the gated policy is as safe as slowing on every blackout (paired difference −0.19 mm, 95% CI [−0.51, +0.19]) and 31.1 mm safer than never slowing, which leaves the ±30 mm lane in 97–100% of runs. On healthy blackouts it never slows, saving 2.6 s per 4 s blackout, with no false alarms in 120 independent scenarios. The analysis predicts the time costs within 1% and the drift within 6% when the fault acts off the arc. The float32 ESP32 port matches the simulator to 0.002 mm, with identical alarm decisions.'),
    ('Conclusion: ', 'Checking the sensors a robot already carries against each other lets it slow down only when a blackout is actually dangerous. The method needs no extra hardware and runs in about 0.1 ms per control step on a low-cost microcontroller.'),
]
KEYWORDS = 'sensor fusion, fault detection, extended Kalman filter, mobile robots, localization loss, speed control'

# ---- Chapter 1
CH(1, 'INTRODUCTION', first=True)
SEC('1.1', 'Background & Overview')
P('Autonomous mobile robots (AMRs) move goods in warehouses along fixed lanes between racks. Many low-cost systems localize them with an overhead camera that tracks a marker on each robot: a camera is cheap, accurate to about a centimetre and easy to install. Its weakness is occlusion. Racks, people and other robots block the view, and the robot loses its position fix for seconds at a time. These blackouts cluster at turns, where the robot passes close to the racks.')
P('A robot does not stop knowing where it is when the camera goes dark. Wheel encoders measure how far each wheel has rolled, and a MEMS gyroscope measures the turn rate. Fusing them in a Kalman filter lets the robot dead-reckon through the blackout, and over a few seconds healthy sensors drift by only a few millimetres. The danger is a sensor that fails during the blackout: a gyro whose bias jumps, a wheel that slips, a worn tyre. The camera is the only external check, and it is unavailable exactly when the fault occurs.')
SEC('1.2', 'Problem Statement')
P('Consider a differential-drive robot that must stay within ±30 mm of its lane centre while it turns behind a rack and loses the camera for up to 4 s. Two simple policies exist. *Never slowing* keeps throughput but lets a faulty sensor drive the robot out of its lane: under a 1°/s gyro bias the lateral error grows as ½vbt^{2} and leaves the lane after 2.6 s at 0.5 m/s. *Slowing on every blackout*, the policy that industrial supervisors use today [1], bounds the damage but charges a time cost on every blackout, faulty or not. Most blackouts are healthy, so most of that cost buys nothing.')
P('This project asks: **can a robot tell, using only its own sensors, which blackouts are dangerous, and slow down only for those?** And if it can, how much safety does it give up, and how much time does it save, compared with slowing every time?')
SEC('1.3', 'Objectives of the Project')
P('The objectives of this project are:')
BUL('To model a camera-localized differential-drive robot, its sensors and realistic sensor faults in a deterministic simulator with paired random streams.')
BUL('To design a consistency monitor that compares the gyro and encoder turn rates during a blackout, and a speed policy that slows only when the monitor raises an alarm.')
BUL('To derive closed-form predictions of drift, detection delay, regret and an admissible threshold range before running any experiment.')
BUL('To test pre-registered hypotheses on safety, time cost and false alarms against never slowing, always slowing and an oracle.')
BUL('To show that the estimator and monitor run on a low-cost microcontroller with results identical to the simulator.')
SEC('1.4', 'Contributions')
BUL('**Consistency-gated slowdown.** A speed policy for localization blackouts that is triggered by gyro–encoder disagreement rather than by the blackout itself, so it costs no time on healthy blackouts.')
BUL('**Closed-form analysis.** Drift envelopes for each fault type, the survival time of a robot that never slows, the detection delay t_{d} = h/b_{f}, the regret of detection-triggered slowing, ½v(1−α)h^{2}/b_{f}, and a design window 0.42° ≤ h ≤ 0.86° for the alarm threshold, derived from encoder resolution, gyro noise and the lane budget.')
BUL('**A pre-registered paired study.** About 11,000 simulated runs with common random numbers, with non-inferiority and equivalence margins fixed in advance and Holm correction for multiple tests.')
BUL('**An embedded implementation.** A float32 ESP32 port that reproduces the simulator to 0.002 mm, together with an open browser simulator and an interactive demonstration.')
SEC('1.5', 'Scope & Organization of the Report')
P('All data in this report are simulated with assumed MEMS-class sensor models; no physical robot was used. The study covers a kinematic differential-drive robot at 0.3–0.7 m/s on a route with one 90° turn, camera blackouts of 0.5–4 s and four types of sensor fault. Chapter 2 reviews prior work. Chapter 3 describes the system and the methods, and Chapter 4 derives the analytical model. Chapter 5 presents the implementation, the experiments and the results. Chapter 6 discusses advantages, limitations and applications, and Chapter 7 concludes.')

# ---- Chapter 2
CH(2, 'LITERATURE REVIEW')
SEC('2.1', 'Historical Evolution')
P('Odometry is the oldest way for a wheeled robot to estimate its pose. Borenstein and Feng [2] showed that its errors are dominated by systematic causes, unequal wheel diameters and an uncertain wheelbase, and corrected them by calibration. In the same year they introduced Gyrodometry [3]: the robot trusts odometry by default and switches to the gyro for heading whenever the two disagree beyond a threshold, as happens when a wheel crosses a bump. Gyrodometry is the first use of gyro–odometry disagreement as a fault signal and the closest ancestor of our monitor.')
P('The Kalman filter turned such sensor combinations into a principled estimate with a known uncertainty. Field robots used it to ride through outages of an external fix: Batavia et al. [4] fused GPS, wheel encoders and a gyro in an extended Kalman filter (EKF) and drove a Pure Pursuit controller [5] through GPS dropouts on an outdoor coverage robot. A theory of filtering with missing measurements followed. Sinopoli et al. [6] showed that a Kalman filter with randomly lost observations has a critical arrival rate below which its error covariance diverges; Huang and Dey [7] extended the result to bursty, Markovian losses, and Kluge et al. [8] to the EKF.')
P('More recent work measured how packet loss and network timing degrade path following on mobile robots [9], [10], and how bursty loss compares with random loss at the same average rate in process control [11]. Supervisory controllers for heavy mobile robots now brake whenever localization is lost [1].')
SEC('2.2', 'Classification of Existing Approaches')
P('Existing ways to handle a localization blackout fall into four groups:')
BUL('**Hold the last command.** The robot repeats its last steering command until the fix returns. It needs no extra sensors, but any change in path curvature during the gap becomes tracking error.')
BUL('**Dead-reckon through the gap.** Encoders and a gyro, alone or in a Kalman filter, carry the estimate [4]. Accuracy is good while the sensors are healthy, but a fault during the gap goes unseen.')
BUL('**Stop or slow on every loss.** A supervisor caps the speed or brakes when the fix times out [1]. This bounds the harm of any fault but pays the time cost on every blackout.')
BUL('**Detect faults by redundancy.** Gyrodometry [3] and residual-based fault detection compare redundant measurements and flag or switch sources. They decide which sensor to trust, not how fast to drive.')
SEC('2.3', 'Summary of Prior Research & Research Gap')
P('Table 2.1 summarizes the closest work. We found no study that uses onboard sensor disagreement to gate the speed of a robot during an external-fix blackout, and none that measures what such gating gives up in safety or saves in time compared with slowing on every loss. The stability results for intermittent filtering [6]–[8] concern covariance divergence over long horizons; they do not tell a robot whether to slow down during a 4 s gap. This project addresses that gap with a policy, an analysis and a controlled comparison.')
TABLE('Table 2.1: Summary of Reviewed Literature and Existing Systems',
      ['Work [Ref]', 'Method', 'Key finding', 'Limitation for this problem'],
      [['Borenstein & Feng, 1996 [2]', 'Calibration of systematic odometry errors', 'Wheel-diameter and wheelbase errors dominate odometry drift', 'Offline calibration; no runtime fault handling'],
       ['Borenstein & Feng, 1996 [3] (Gyrodometry)', 'Switch heading source when gyro and odometry disagree', 'Removes bump-induced heading errors', 'Chooses a sensor; does not act on speed or blackouts'],
       ['Batavia et al., 2002 [4]', 'EKF (GPS, encoders, gyro) and Pure Pursuit through GPS outages', 'Field robot keeps its coverage paths through dropouts', 'No fault injection or policy comparison'],
       ['Sinopoli et al. [6]; Huang & Dey [7]; Kluge et al. [8]', 'Kalman filter and EKF stability under random and Markov loss', 'Critical arrival rate for bounded covariance', 'Long-horizon stability, not lane safety in a short gap'],
       ['Carbonell et al. [9]; Ferrari et al. [10]; Liu et al. [11]', 'Packet loss and network timing in control loops', 'Loss and timing degrade tracking; bursts differ from random loss', 'No onboard fault detection or speed policy'],
       ['Heydari Shahna et al., 2026 [1]', 'Supervisor brakes on any localization fault, including a pose timeout', 'Safe goal reaching on a 6 t robot', 'One policy; the cost of always braking is not measured']],
      [1900, 2250, 2300, 2219])

# ---- Chapter 3
CH(3, 'SYSTEM DESIGN & METHODOLOGY')
SEC('3.1', 'Block Diagram & Working Principle')
P('Figure 3.1 shows the closed loop. An overhead camera reports the robot’s pose ten times per second, and a loss channel removes reports during a blackout. Wheel encoders and a gyro report at 50 Hz. An EKF fuses all three into a pose estimate, and Pure Pursuit steers that estimate along the route at 10 Hz. Two blocks are new: a **consistency monitor**, which compares the gyro with the encoders, and a **speed policy**, which caps the speed when the monitor raises an alarm during a blackout. Scoring uses the true pose, which the robot never sees.')
P('The working principle rests on one observation. The gyro and the encoder pair are two independent measurements of the same quantity, the robot’s turn rate. While both are healthy their difference is small noise. A gyro bias, a slipping wheel or a wrong wheel radius makes them disagree. During a blackout the camera cannot help, but this disagreement remains observable, so the robot can check itself.')
FIGURE(RFIG('fig_architecture.png'), 6.0, 'Figure 3.1: System architecture. The blue blocks are the proposed consistency monitor and speed policy; the other blocks are standard.')
SEC('3.2', 'Simulated Robot, Sensors & Faults')
P('The robot is a differential drive with wheelbase b = 0.20 m and wheel radius r = 33 mm. Each wheel motor is a first-order lag (τ = 0.12 s) saturating at 1.0 m/s, and the wheel travel within each 10 ms step is integrated exactly, which makes the results independent of the step size (Section 5.3). The route is a 3 m straight, a 90° left arc of radius R = 1 m and a 5 m straight. The sensors are modelled as follows:')
BUL('**Encoders:** 360 counts per revolution (0.576 mm of travel per count) at 50 Hz. They count wheel rotation, so slip and radius errors are invisible to them.')
BUL('**Gyro:** 50 Hz, reporting the mean turn rate over each sample interval, as an integrating MEMS sensor does, plus angle random walk (0.015°/√s), a Gauss–Markov bias (σ = 0.004°/s, τ = 100 s), a warm-up drift and a turn-on bias that a 1 s stationary calibration removes. These values are typical of MEMS gyros characterized by Allan variance [12].')
BUL('**Camera:** 10 Hz pose with standard deviations of 10 mm in position and 1° in heading.')
BUL('**Loss channel:** deterministic blackouts of 0.5–4 s that start at a chosen route phase (straight, turn entry, mid-turn or turn exit), and a two-state Gilbert channel [13] for bursty loss.')
P('Four faults are injected: a gyro bias step of +0.5 or +1°/s that starts 0.5 s into the blackout; a 10% slip of the right wheel for 0.5 s; both wheel radii 3% too large for the whole run; and a ±1% radius difference between the two wheels.')
SEC('3.3', 'System Parameters')
P('Table 3.1 lists the parameters. All of them were fixed before the scored experiments; the monitor parameters were frozen after a separate calibration experiment (Section 5.5).')
TABLE('Table 3.1: Robot, Sensor, Controller and Monitor Parameters',
      ['Block', 'Symbol', 'Value', 'Note'],
      [['Route', 'L_{1}, R, L_{3}', '3.0 m, 1.0 m, 5.0 m', 'R = 0.55–1.5 m in Part C'],
       ['Robot', 'b, r', '0.20 m, 33 mm', 'small AMR'],
       ['Speed', 'v', '0.5 m/s', '0.2 m/s in Part A; 0.3–0.7 m/s in Part C'],
       ['Motors', 'τ, u_{max}', '0.12 s, 1.0 m/s', 'exact discretization'],
       ['Encoders', 'N_{c}, res', '360 CPR, 0.576 mm', '17.4 counts per 20 ms at 0.5 m/s'],
       ['Gyro', 'ARW; bias', '0.015°/√s; σ = 0.004°/s, τ = 100 s', 'assumed MEMS class'],
       ['Camera', 'σ_{xy}, σ_{θ}, rate', '10 mm, 1°, 10 Hz', 'no latency'],
       ['Pure Pursuit', 'L_{d}, rate', '0.30 m, 10 Hz', '0.15 m in Part A'],
       ['Monitor', 'T_{w}, h, g, τ_{m}', '2 s, 0.6°, 0.3 s, 10 s', 'frozen before scored runs'],
       ['Policy', 'α', '0.3 (cap 0.15 m/s)', 'same for SB, SA and SO'],
       ['Safety', 'e_{tol}', '±30 mm', 'fixed in advance']],
      [1500, 1700, 2900, 2569])
SEC('3.4', 'State Estimation')
P('The EKF state is **x** = [x, y, θ, b_{g}]^{T}: position, heading and gyro bias. At 50 Hz the filter predicts with the encoder distance Δs = (d_{L} + d_{R})/2 and the bias-corrected gyro rate (Section 4.1). When a camera report arrives, it corrects position and heading. The bias state lets the filter learn slow gyro drift while the camera is available. Its process noise q_{b} = (3×10^{−4})^{2} per step was chosen by a rule fixed before tuning: the smallest value for which the camera-available reference recovers from a +1°/s bias step with less than 10 mm extra error, while the normalized estimation error squared (NEES) stays in [1, 3.5].')
P('Two simpler estimators serve as baselines. **HOLD** repeats the last steering command during a gap. **DR** (dead reckoning) resets its pose to each camera report and integrates the encoders and the calibrated gyro in between, without a filter.')
SEC('3.5', 'Path Tracking')
P('Pure Pursuit [5] steers toward a target point a fixed lookahead L_{d} ahead of the projection of the *estimated* pose on the route. With y_{g} the lateral offset of the target in the robot frame and d its distance, the commanded turn rate is ω_{c} = 2v_{c}y_{g}/d^{2}. L_{d} is fixed within each experiment, so capping the speed changes only the speed, not the curvature command.')
SEC('3.6', 'Consistency Monitor')
P('The monitor forms the turn-rate residual r_{k} = ω_{g} − ω_{enc}, where ω_{enc} = (d_{R} − d_{L})/(bΔt) is the turn rate measured by the encoders. It subtracts an offset o, a 10 s low-pass filter of r_{k} that is updated only while the camera is fresh and frozen during blackouts, and sums over a sliding window of T_{w} = 2 s:')
EQ(r'D(t)=\sum_{k\in[t-T_w,t]}(r_k-o)\Delta t', '3.1')
P('D is the heading disagreement accumulated over the last 2 s, measured relative to the disagreement seen before the blackout. An alarm fires when |D| > h = 0.6° and the camera has been missing for more than g = 0.3 s; it clears with the next camera report. Summing over a window, instead of thresholding the raw rate, is essential: the per-sample encoder turn rate is quantized in steps of 8.3°/s, but the windowed heading error stays within two encoder counts (0.33°) whatever the window length.')
P('An earlier version of the monitor referenced the EKF’s bias estimate instead of its own offset. Calibration showed that it inherited the filter’s tuning noise and raised false alarms, which triggered a pre-registered stop criterion. The current design was derived from that diagnosis before any scored run (Section 5.5).')
SEC('3.7', 'Speed Policies')
P('Table 3.2 defines the four speed policies. All of them use the same EKF, speed fraction α = 0.3, gate g = 0.3 s and release rule (full speed again at the next camera report). The oracle SO knows the true fault time, which no real robot does; it shows the best that any detector could achieve. A fifth arm, DR, drives at full speed on plain dead reckoning and isolates the value of the filter.')
TABLE('Table 3.2: Speed Policies Compared',
      ['Policy', 'Speed during a blackout', 'Deployable'],
      [['N (never slow)', 'v', 'yes'],
       ['SB (slow on every blackout)', 'αv once the camera has been missing for g', 'yes'],
       ['SA (slow on alarm, proposed)', 'αv while the monitor alarm is on', 'yes'],
       ['SO (oracle)', 'αv from the true fault onset (after g)', 'no: reference bound']],
      [3000, 3769, 1900])

# ---- Chapter 4
CH(4, 'MATHEMATICAL MODELING & ANALYSIS')
SEC('4.1', 'Governing Equations')
P('All variables are defined below; angles are in radians inside the equations and quoted in degrees in the text.')
BUL('**Nomenclature:** x, y, θ = robot position (m) and heading; v, ω = forward speed (m/s) and turn rate (rad/s); b = wheelbase (m); r = wheel radius (m).')
BUL('d_{L}, d_{R} = wheel distances counted by the encoders in one step (m); ω_{g} = gyro output (rad/s); b_{g} = gyro bias; b_{f} = size of a gyro fault (rad/s); Δt = 20 ms sensor step.')
BUL('e = signed cross-track error of the true pose (m); e_{tol} = 30 mm lane half-width; T_{b} = blackout length (s); T_{w} = monitor window (s); h = alarm threshold (rad); α = speed fraction during slowing.')
P('The robot follows the unicycle kinematics')
EQ(r'\dot{x}=v\cos\theta,\quad \dot{y}=v\sin\theta,\quad \dot{\theta}=\omega', '4.1')
EQ(r'v=\frac{g_R+g_L}{2},\quad \omega=\frac{g_R-g_L}{b},\quad g_i=\dot{\phi}_i r(1+\varepsilon_i)(1-s_i)', '4.2')
P('where g_{i} is the ground speed of wheel i, φ̇_{i} its rotation rate, ε_{i} a radius error and s_{i} a slip fraction. The encoders count φ_{i}, so ε_{i} and s_{i} never appear in their output. The gyro measures')
EQ(r'\omega_g=\bar{\omega}+b_0+b_w(t)+b_{GM}+b_f u(t-t_f)+\eta', '4.3')
P('where ω̅ is the mean true turn rate over the sample, b_{0} the turn-on bias, b_{w} the warm-up drift, b_{GM} the Gauss–Markov bias, u(·) the unit step of a fault at time t_{f} and η white noise. The EKF predicts')
EQ(r'x^-=x+\Delta s\cos\theta,\quad y^-=y+\Delta s\sin\theta,\quad \theta^-=\theta+(\omega_g-b_g)\Delta t', '4.4')
P('while the bias b_{g} is held constant, and corrects with the camera pose through the standard EKF update with measurement matrix H = [I_{3} 0].')
SEC('4.2', 'Drift Under Each Fault')
P('During a blackout the filter’s heading comes from the gyro. A gyro bias b_{f} therefore rotates the estimated heading at rate b_{f}, and Pure Pursuit, which keeps the *estimate* on the route, steers the true robot off it. For small angles on a straight path, the lateral error after an exposure time t is')
EQ(r'e(t)\approx\frac{1}{2}v b_f t^2', '4.5')
P('Table 4.1 lists the envelope and the monitor’s response for each fault. Two cases stand out. A uniform radius error does nothing on a straight, because heading comes from the gyro, but in a turn the robot drives a circle of radius R(1+ε). And a radius *difference* between the wheels barely affects tracking, yet makes the sensors disagree: a harmless fault that the monitor flags anyway.')
TABLE('Table 4.1: Predicted Effect of Each Fault During a Blackout',
      ['Fault', 'Tracking effect', 'Depends on route phase?', 'Monitor residual |D|'],
      [['Gyro step b_{f}', '½ v b_{f} t^{2}', 'no', 'b_{f} · min(t, T_{w})'],
       ['Gyro noise (ARW N)', 'σ_{e} ≈ v N t^{3/2}/√3', 'no', 'σ_{D} = N√T_{w} = 0.021°'],
       ['Both wheels +ε', 'outward εR(1 − cos φ_{b})', 'zero on straights', '≈ ε|Δθ| (turns only)'],
       ['Wheels −δ/2, +δ/2', '≈ 0', 'no', '≈ vδT_{w}/b = 5.7°'],
       ['Slip s for T_{s}', 'real yaw disturbance, seen by the gyro', 'weak', '≈ s v T_{s}/b = 7.2°']],
      [2000, 2600, 1800, 2269])
SEC('4.3', 'Survival Time & Detection Delay')
P('Setting e(t) = e_{tol} in (4.5) gives the time for which a robot that never slows survives a gyro fault:')
EQ(r'T^*=\sqrt{\frac{2e_{tol}}{v b_f}}', '4.6')
P('At v = 0.5 m/s this is 2.62 s for b_{f} = 1°/s and 3.71 s for 0.5°/s, so a 4 s blackout is long enough to leave the lane. The monitor residual grows as b_{f}t until the window is full, so it crosses the threshold after')
EQ(r't_d=\frac{h}{b_f},\qquad b_f>b_{min}=\frac{h}{T_w}', '4.7')
P('which is 0.60 s for 1°/s. A fault smaller than b_{min} = 0.3°/s cannot fill the window enough to trigger an alarm; for such faults the threshold ceiling of Section 4.5 guarantees that the error stays inside the lane.')
SEC('4.4', 'Cost & Regret of Each Policy')
P('Integrating the drift rate v(t)b_{f}(t − t_{f}) over a fault exposure T gives the error of each policy:')
EQ(r'e_N=\frac{1}{2}v b_f T^2,\quad e_{SO}=\alpha e_N,\quad e_{SA}=\frac{1}{2}v b_f(\alpha T^2+(1-\alpha)t_d^2)', '4.8')
P('SA drives at full speed only during the detection delay. Its regret against the oracle is therefore')
EQ(r'e_{SA}-e_{SO}=\frac{1}{2}v(1-\alpha)\frac{h^2}{b_f}', '4.9')
P('Regret falls as 1/b_{f}: large faults are detected fast and cost little extra. It is largest just above b_{min} (3.7 mm) and only 1.1 mm at 1°/s. The time cost of driving at αv for a duration D_{s} is (1 − α)D_{s}. SB pays (1 − α)(T_{b} − g) on every blackout; SA pays nothing on a healthy one.')
SEC('4.5', 'Threshold Design Window')
P('The threshold has a floor set by sensor noise and a ceiling set by safety:')
EQ(r'h_{min}=\frac{2 res}{b}+4N\sqrt{T_w}=0.42°,\quad h_{max}=\frac{2e_{tol}T_w}{v T_{max}^2}=0.86°', '4.10')
P('The floor is two encoder counts of left–right difference (0.33°) plus four standard deviations of gyro noise over the window (0.08°); below it, quantization alone triggers alarms. The ceiling guarantees that any gyro fault too small to be detected (b_{f} < h/T_{w}) also keeps the robot inside the lane for the longest blackout, T_{max} = 4 s. We chose h = 0.6°, inside the window. Figure 4.1 shows that the healthy residual measured in calibration stayed well below it.')
FIGURE(FIG('fig_threshold_window.png'), 5.2, 'Figure 4.1: Threshold design window. Histogram of the largest healthy residual |D| in 480 calibration runs, against the quantization floor, the window [h_{min}, h_{max}] and the frozen threshold h = 0.6°.')
SEC('4.6', 'Design Calculations & Numerical Case Study')
P('Table 4.2 evaluates (4.5)–(4.9) for the primary experiment: v = 0.5 m/s, a +1°/s gyro fault starting 0.5 s into the blackout, α = 0.3 and h = 0.6°. These values were computed before any scored run and are compared with measurements in Section 5.6.')
TABLE('Table 4.2: Predicted Excess Error and Time Lost in the Primary Experiment',
      ['Blackout T_{b}', 'Policy', 'Excess error (mm)', 'Time lost, healthy (s)', 'Time lost, fault (s)'],
      [['4 s', 'N', '53.5', '0', '0'], ['4 s', 'SB', '16.0', '2.59', '2.59'], ['4 s', 'SA', '17.1', '0', '2.03'], ['4 s', 'SO', '16.0', '0', '2.45'],
       ['2 s', 'N', '9.8', '0', '0'], ['2 s', 'SB', '2.9', '1.19', '1.19'], ['2 s', 'SA', '4.0', '0', '0.63'], ['2 s', 'SO', '2.9', '0', '1.05']],
      [1500, 1300, 1950, 1960, 1959])
P('The table makes two predictions. For 4 s blackouts, SA should be almost as safe as SB (1.1 mm worse) while losing no time on healthy blackouts. For 2 s blackouts even N stays in the lane, because the exposure is shorter than T^{*} plus the fault delay (about 3.1 s); there, slowing buys nothing and SB’s time cost is pure loss.')

# ---- Chapter 5
CH(5, 'IMPLEMENTATION, TESTING & RESULTS')
SEC('5.1', 'Implementation')
P('**Simulator.** The simulator is written in JavaScript (14 modules) and runs in a web browser; batch experiments run in a background Web Worker. Each sensor has its own seeded random stream, and every policy sees the same streams for the same seed (common random numbers), so differences between policies come from the policy alone. The full experimental grid runs in under two minutes.')
P('**Analysis.** Python scripts (NumPy, SciPy, Matplotlib) read the saved results, run the pre-registered tests and draw every figure. Every number in this report traces to a results file and a code commit.')
P('**Embedded port.** The EKF, the monitor, Pure Pursuit and the route were ported to float32 C++ for an ESP32 microcontroller and run in the Wokwi simulator [14] (Figure 5.1). The firmware replays the sensor inputs of two recorded simulator runs, one healthy and one with a gyro fault, recomputes every estimate and command on the chip and compares them with the simulator at every 20 ms step. Four LEDs show the wheel commands, the camera status and the alarm; a button injects a gyro fault live.')
FIGURE(RFIG('fig_wokwi.png'), 6.0, 'Figure 5.1: ESP32 replay fixture as wired in Wokwi (drawn from the project’s diagram.json).')
P('**Demonstration.** An interactive web page replays any scenario with all five policies side by side, showing the true and estimated robot, the lane, the monitor residual and each speed decision. The code, data and demonstration are public [15].')
SEC('5.2', 'Experimental Setup & Testing Procedure')
P('The experiments, hypotheses, metrics and decision rules were written down before any scored run. Table 5.1 lists the experiments.')
TABLE('Table 5.1: Experiments',
      ['ID', 'Question', 'Factors', 'Arms', 'Runs'],
      [['V', 'Is the simulator correct?', '12 configurations and all baselines', 'all', '~200'],
       ['A', 'Do onboard sensors help on short gaps?', '3 loss patterns × 3 phases × 2 faults; v = 0.2 m/s', 'HOLD, DR, EKF', '1,080'],
       ['CAL', 'Freeze the alarm threshold', '4 blackout lengths × 4 phases; healthy; separate seeds', 'EKF', '480'],
       ['B0', 'Main test (P1–P4)', 'T_{b} ∈ {2, 4} s × {entry, mid} × {healthy, gyro +1°/s}', 'N, SB, SA, SO', '960'],
       ['B', 'Which fault hurts where?', '4 lengths × 4 phases × 6 fault conditions', 'DR, N, SB, SA, SO', '4,800'],
       ['C', 'Do the formulas scale?', 'R × v; gyro fault 0.2–2°/s; lookahead rule', 'N, SA, SO', '900'],
       ['D', 'Does burstiness matter?', 'Gilbert channel, 10% loss, mean burst 1–16', 'HOLD, DR, EKF', '1,200'],
       ['S', 'Sensitivity', '2× gyro noise, 2× camera noise, α, 50 Hz steering', 'B0 arms', '1,280']],
      [700, 2300, 3169, 1600, 900])
P('**Metrics.** The primary metric is the excess peak cross-track error e_{ex}: the largest |e| of the true robot in a window from the blackout start to 3 s after the camera returns, minus the same quantity in a reference run with the same seed and no camera loss. A *breach* is any |e| > 30 mm. The time cost is the extra route completion time relative to the reference.')
P('**Hypotheses.** P1 (safety): SA is non-inferior to SB, meaning that the upper 95% bound of the paired median of e_{ex}^{SA} − e_{ex}^{SB} lies below 5 mm. P2 (time): SA completes healthy runs faster than SB. P3 (benefit): under the fault, SA has less error than N. P4 (false alarms): no alarm in healthy runs. The primary cell is a 4 s blackout starting at turn entry.')
P('**Statistics.** The unit of analysis is the seed; time steps are never treated as samples. We report the median of paired differences with a 95% percentile bootstrap interval over seeds (10,000 resamples) [16], test superiority with a paired sign-flip permutation test and correct P1–P3 with Holm’s method [17]. Equivalence uses two one-sided tests [18], and false-alarm rates use Clopper–Pearson bounds [19]. Common random numbers make the paired variance tiny, so decisions rest on margins fixed in advance (5 mm for safety, 3 mm for equivalence), not on p-values alone.')
SEC('5.3', 'Verification')
P('Before any scored run, the simulator passed the checks in Table 5.2. V6 uses the Allan variance [12]; V7 confirms the drift law (4.5) to within 4% on a straight path.')
TABLE('Table 5.2: Verification Checks',
      ['Check', 'Pass criterion', 'Result'],
      [['V1 Determinism', 'same seed gives identical output', 'identical'],
       ['V2 Loss schedule', 'dropped reports equal the schedule', 'exact for all patterns'],
       ['V3 No-loss baseline', 'peak error < 15 mm', 'max 14.9 mm'],
       ['V4 Step-size convergence', '|Δe_{ex}| < 1 mm between 10 ms and 5 ms', '0.001 mm'],
       ['V5 EKF consistency', 'mean NEES in [1, 3.5]', '1.90'],
       ['V6 Gyro model', 'Allan deviation recovers ARW within 10%', '0.04% error'],
       ['V7 Drift envelopes', 'within 10% of (4.5) on a straight', '≤ 3.9%'],
       ['V8 Firmware parity', 'pose < 1 mm, identical alarms', '0.002 mm, identical'],
       ['V10 Window coverage', 'no truncated scoring window', '0 of 9,500']],
      [2600, 3700, 2369])
P('Verification caught two modelling errors before they could affect any result. A bias state with too little process noise made the no-loss reference itself drift by 60 mm, and point-sampling the gyro created a spurious 0.29° heading error at every change of curvature. Both were fixed (Sections 3.2 and 3.4) before any scored run.')
SEC('5.4', 'Short Gaps: Holding, Dead Reckoning & the EKF')
P('On short gaps (eight missing reports at 0.2 m/s), all three estimators stay well inside the lane (Figure 5.2). Holding the last command costs 5–10 mm, most mid-turn, because the held command was computed from a noisy fix. Dead reckoning and the EKF track almost equally well: their difference is within ±3 mm in 13 of 18 cells. The EKF’s advantage lies elsewhere. Its estimate error is 4.5–7.3× smaller and its steering jump when the camera returns is 3.7–21× smaller, because it filters camera noise instead of resetting to each noisy fix.')
FIGURE(FIG('fig_estimators_short_gaps.png'), 6.0, 'Figure 5.2: Short gaps (Part A, 20 seeds). HOLD, DR and the EKF across loss patterns and route phases: tracking error, estimate error and steering jump.')
SEC('5.5', 'Monitor Calibration')
P('The threshold was frozen on 480 healthy runs with seeds that were never scored. The first monitor design failed. It referenced the EKF bias estimate, which wanders by 0.09°/s once the robot moves, and it raised 7 false alarms in 480 runs. The freeze rule then required h = 0.95°, above the ceiling h_{max}, which is a pre-registered stop criterion. The data pointed to the cause, so the monitor was given its own offset (Section 3.6) and calibration was rerun on the same calibration seeds. The redesigned monitor’s largest healthy residual was 0.318°, at the quantization floor of 0.33° predicted by (4.10), with no false alarms. The threshold stayed at the pre-registered h = 0.6°, a 1.9× margin (Figure 4.1).')
SEC('5.6', 'Main Result: Selective Versus Blanket Slowdown')
P('Table 5.3 reports the four primary tests in the primary cell (4 s blackout from turn entry, 30 paired seeds). All four hypotheses are supported.')
TABLE('Table 5.3: Primary Hypothesis Tests (B0, Primary Cell)',
      ['Test', 'Contrast', 'Measured [95% CI]', 'Predicted', 'Verdict'],
      [['P1 safety', 'SA − SB excess error, fault', '−0.19 mm [−0.51, +0.19]', '+1.1 mm', 'non-inferior (bound < 5 mm)'],
       ['P2 time', 'SB − SA completion time, healthy', '+2.60 s [2.60, 2.60]', '+2.59 s', 'supported'],
       ['P3 benefit', 'N − SA excess error, fault', '+31.1 mm [28.4, 33.8]', '+36.4 mm', 'supported'],
       ['P4 false alarms', 'alarms in healthy scenarios', '0/120 (upper bound 2.5%)', '0', 'supported']],
      [1350, 2350, 2150, 1100, 1719])
P('SA matches SB’s safety to within half a millimetre. Without slowing, the robot leaves the lane in 100% of fault runs that start at turn entry and in 97% of those that start mid-turn. With SA, SB or SO it stays inside in every turn-entry run and in 87–90% of mid-turn runs. The monitor raised its alarm 0.50–0.54 s after the fault (predicted 0.60 s), always before the camera returned. On healthy blackouts SA never slowed, so it saved the full 2.6 s that SB loses per 4 s blackout. Figure 5.3 shows one such pair of runs, and Figure 5.4 the trade-off over all seeds.')
FIGURE(FIG('fig_viva_timeline.png'), 6.0, 'Figure 5.3: One 4 s blackout from turn entry with the same seed, healthy (left) and with a +1°/s gyro fault (right): camera and alarm, cross-track error of N, SB and SA, monitor residual D against ±h, and the wheel speeds of SA.')
FIGURE(FIG('fig_policy_tradeoff.png'), 5.6, 'Figure 5.4: Safety against time cost for 4 s blackouts (B0). Under a fault SA reaches the error of SB, and on healthy blackouts it loses no time.')
P('For 2 s blackouts every policy stayed within 2 mm of the reference, as the case study predicted: slowing buys nothing there, yet SB still loses 1.2 s per blackout.')
P('**Prediction accuracy.** Table 5.4 compares the closed-form model with the measurements (4 s blackouts unless stated). Time costs match within 1%. For drift we use the path-aligned deviation e_{dev}, a secondary metric added after pre-registration: the pre-registered e_{ex} subtracts a reference peak whose sign can oppose the drift, so it does not measure drift alone. With mid-turn onset, where most of the exposure falls on the exit straight, the model matches within 6%. With turn-entry onset, where the exposure falls on the arc, the straight-path formula overpredicts by 15–34%, because Pure Pursuit partly absorbs the heading error along a curve.')
TABLE('Table 5.4: Closed-Form Predictions Versus Measurements',
      ['Quantity', 'Predicted', 'Measured'],
      [['SB time lost, healthy', '2.59 s', '2.60 s'], ['SA time lost, fault', '2.03 s', '2.04–2.10 s'], ['SO time lost, fault', '2.45 s', '2.46 s'],
       ['SB time lost, healthy, 2 s blackout', '1.19 s', '1.20 s'], ['Alarm delay after the fault', '0.60 s', '0.50–0.54 s'],
       ['Drift e_{dev}, mid-turn onset (N / SA / SB / SO)', '53.5 / 17.1 / 16.0 / 16.0 mm', '50.5 / 17.2 / 16.8 / 16.9 mm'],
       ['Drift e_{dev}, turn-entry onset (N / SA / SB / SO)', '53.5 / 17.1 / 16.0 / 16.0 mm', '43.6 / 11.3 / 13.7 / 13.3 mm']],
      [3900, 2400, 2369])
SEC('5.7', 'Fault Map')
P('Figure 5.5 maps the error of each policy over fault type and route phase (4,800 runs). Three patterns match Table 4.1. Gyro faults hurt at every phase (46–57 mm for N at 1°/s). A 3% radius error is invisible on the straight (12 mm, as when healthy) and large in the turn (53 mm at turn entry). A ±1% radius difference does not affect tracking. Across every fault cell, SA stayed within ±2.5 mm of SB.')
FIGURE(FIG('fig_fault_map.png'), 6.0, 'Figure 5.5: Fault map (Part B). Path-aligned deviation for each policy over fault type and blackout onset phase, 4 s blackouts.')
P('Three further findings were not anticipated. First, plain dead reckoning fails on long blackouts even without a fault: it starts from the last noisy camera fix (σ_{θ} = 1°), and vTσ_{θ} ≈ 35 mm over 4 s, so it leaves the lane in 30–70% of healthy runs while the EKF stays within 6–14 mm. Second, the harmless ±1% radius difference triggers SA in every blackout, so SA slows exactly as SB does: the monitor cannot tell a harmless disagreement from a harmful one. Third, a 3% radius error breaches the lane at turn entry for every policy, the oracle included. Slowing cannot fix a geometric error; only calibration can [2].')
SEC('5.8', 'Scaling, Detection Delay & Regret')
P('The gyro-fault error grows linearly with speed and does not depend on the turn radius, as (4.5) predicts (Figure 5.6), but it reaches only 0.80–0.85 of the straight-path value because the exposure falls on the arc. The detection delay follows h/b_{f} closely and slightly early: 2.16, 0.94, 0.50 and 0.26 s for b_{f} = 0.2, 0.5, 1 and 2°/s (Figure 5.7). Even 0.2°/s, below the predicted b_{min}, was detected, because quantization noise helps the first threshold crossing; the effective b_{min} is about 0.15°/s. The regret of SA against the oracle stayed within ±1.5 mm for every fault size, below what the closed loop can resolve. In practice the monitor performs like an oracle.')
FIGURE(FIG('fig_scaling.png'), 6.0, 'Figure 5.6: Scaling (Part C1). Error of N after a 4 s blackout from turn entry against speed, for three turn radii: gyro fault (left) and wheels 3% too large (right).')
FIGURE(FIG('fig_detection_regret.png'), 6.0, 'Figure 5.7: Detection delay and regret against gyro fault size (Part C2). The delay follows h/b_{f}, and SA stays within 1.5 mm of the oracle for every fault size.')
SEC('5.9', 'Burstiness')
P('At a matched 10% average loss rate, longer bursts hurt only the robot without onboard sensing (Figure 5.8). HOLD’s peak error grows from 11.6 to 39.9 mm as the mean burst length grows from 1 to 16 reports, and it leaves the lane in up to 55% of runs. DR and the EKF stay flat, at 11–13 mm and 8.0–8.6 mm. With encoders and a gyro, the pattern of the loss matters far less than its total length.')
FIGURE(FIG('fig_burstiness.png'), 5.6, 'Figure 5.8: Burstiness (Part D, 100 seeds). Peak error against mean burst length at a matched 10% loss rate.')
SEC('5.10', 'Sensitivity')
P('Table 5.5 repeats the primary cell (4 s blackout from turn entry, 10 seeds) under harsher conditions and reports the median excess error.')
TABLE('Table 5.5: Sensitivity of the Main Result',
      ['Variant', 'N (mm)', 'SA (mm)', 'SB (mm)', 'Healthy false alarms'],
      [['Gyro noise ×2', '38.5', '7.1', '7.6', '0'], ['Camera noise ×2', '34.9', '3.0', '2.9', '0'], ['Steering at 50 Hz', '39.1', '5.4', '5.7', '0']],
      [2869, 1350, 1350, 1350, 1750])
P('The main claim survives every variant: SA stays within ±0.7 mm of SB, loses no time on healthy blackouts and raises no false alarms, and its alarm delay stays at 0.48–0.51 s. The speed fraction α sets the trade-off for both slowing policies. At α = 0.2, SA reaches 1.5 mm and loses 2.39 s under a fault; at α = 0.5, 16.9 mm and 1.50 s. SA keeps its zero healthy cost at every α.')
SEC('5.11', 'Embedded Parity')
P('The firmware replays two traces of 706 steps of 20 ms each (Section 5.1). Table 5.6 compares its outputs with the simulator at every step.')
TABLE('Table 5.6: ESP32 Firmware Versus Simulator',
      ['Pass', 'Max pose difference', 'Max heading difference', 'Alarm steps differing', 'Max wheel-command difference', 'Result'],
      [['Healthy', '0.0024 mm', '0.00018°', '0 / 706', '3×10^{−6} m/s', 'PASS'],
       ['Gyro fault', '0.0011 mm', '0.00015°', '0 / 706', '1×10^{−6} m/s', 'PASS']],
      [1200, 1500, 1550, 1450, 1969, 1000])
P('The float32 firmware reproduces the double-precision simulator to micrometres, and its alarm fires at the same step (8.04 s, 0.44 s after the fault). One control step (EKF, monitor and Pure Pursuit) takes 38–127 µs on average and at most 450 µs, under 3% of the 20 ms budget. Before the microcontroller test, a bit-exact replay in JavaScript confirmed that the recorded traces carry every input the firmware needs.')
SEC('5.12', 'Discussion')
P('Three conclusions follow. First, a robot can tell dangerous blackouts from harmless ones with sensors it already has: gating the slowdown on gyro–encoder disagreement keeps the safety of always slowing and removes its cost on healthy blackouts. Second, the value of slowing depends on the blackout length: below about 3 s at 0.5 m/s even a faulty robot stays in the lane, and above it slowing is essential. Third, a simple closed-form model predicts the time costs almost exactly and the drift to within 6–34%, which makes it usable for design: given a lane width and the longest expected blackout, (4.10) gives the admissible threshold range before any experiment is run.')

# ---- Chapter 6
CH(6, 'ADVANTAGES, LIMITATIONS & APPLICATIONS')
SEC('6.1', 'Advantages of the Proposed System')
BUL('**No extra hardware.** The monitor uses the gyro and encoders that the robot already carries for dead reckoning.')
BUL('**No time cost on healthy blackouts.** The robot saved 2.6 s per 4 s blackout relative to blanket slowing, with the same safety under a gyro fault.')
BUL('**Cheap to run.** A monitor update is one subtraction and a windowed sum; the complete estimator, monitor and controller step takes about 0.1 ms on average on an ESP32.')
BUL('**Designable from datasheets.** The threshold window (4.10) follows from encoder resolution, gyro noise, lane width and the longest blackout.')
SEC('6.2', 'Limitations')
BUL('**Simulation only.** All results come from a simulator with assumed MEMS-class sensor models. Agreement between the analysis and the simulator verifies one model; it does not validate it against a physical robot.')
BUL('**Faults the monitor cannot see.** A uniform wheel-radius error shows up only when the turn rate changes and cannot be fixed by slowing; it needs calibration.')
BUL('**Harmless disagreements.** A small left–right radius difference triggers slowdowns that do not improve safety.')
BUL('**Scope.** One route with a single 90° turn, a kinematic robot at 0.3–0.7 m/s, step-shaped gyro faults and no camera latency.')
BUL('**Offset absorption.** The monitor offset absorbs persistent disagreement over about 30 s of camera availability, so a fault that begins long before a blackout may go unflagged.')
SEC('6.3', 'Real-World Applications')
BUL('**Warehouse AMRs** localized by overhead cameras or ceiling markers, where racks cause predictable blackouts at turns.')
BUL('**Robots with intermittent external fixes**, such as UWB or motion-capture tracking in factories and laboratories, or GPS outages under trees and bridges.')
BUL('**Safety supervisors** that today brake on every localization timeout and could instead brake only when the onboard sensors disagree.')

# ---- Chapter 7
CH(7, 'CONCLUSION & FUTURE SCOPE')
SEC('7.1', 'Conclusion')
P('This project asked whether a robot that loses its camera fix can decide for itself when to slow down. We built a consistency monitor that compares the gyro with the wheel encoders, and a policy that slows only when they disagree. Before running experiments, we derived how far a faulty robot drifts, how fast the monitor detects the fault, what the detection delay costs and which thresholds are admissible. In about 11,000 paired simulation runs the gated policy matched the safety of slowing on every blackout (−0.19 mm difference) and saved 2.6 s on each healthy 4 s blackout, with no false alarms, while never slowing left the lane in 97–100% of fault runs. The analysis predicted the time costs within 1%, and a float32 ESP32 implementation matched the simulator to 0.002 mm. All five objectives of Section 1.3 were met, within the limits of a simulation study.')
SEC('7.2', 'Future Scope')
BUL('**Real-robot validation:** run the monitor on a small differential-drive robot, or on public logs with raw gyro and encoder data, and measure false alarms under real noise, vibration and wheel wear.')
BUL('**Curved-path analysis:** extend (4.5) to exposure on an arc, where the measured drift is 0.75–0.85 of the straight-path value.')
BUL('**Sequential detection:** replace the windowed sum with a CUSUM test [20] to shorten the detection delay at the same false-alarm rate.')
BUL('**Graded response:** scale the speed with the size of the disagreement instead of one fixed cap, and separate harmful from harmless disagreements by their pattern.')
BUL('**Wider conditions:** camera latency, ramp-shaped gyro drift, more route geometries and higher speeds.')

# ---- References
REFS = [
    'M. Heydari Shahna, P. Mustalahti, and J. Mattila, “Robust vision-based goal-reaching control for mobile robots using a hierarchical learning framework,” *Robotics and Autonomous Systems*, Art. no. 105710, 2026, doi: 10.1016/j.robot.2026.105710.',
    'J. Borenstein and L. Feng, “Measurement and correction of systematic odometry errors in mobile robots,” *IEEE Trans. Robot. Autom.*, vol. 12, no. 6, pp. 869–880, Dec. 1996.',
    'J. Borenstein and L. Feng, “Gyrodometry: A new method for combining data from gyros and odometry in mobile robots,” in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, 1996, pp. 423–428.',
    'P. Batavia, S. Roth, and S. Singh, “Autonomous coverage operations in semi-structured outdoor environments,” in *Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS)*, 2002.',
    'R. C. Coulter, “Implementation of the pure pursuit path tracking algorithm,” Robotics Inst., Carnegie Mellon Univ., Pittsburgh, PA, USA, Tech. Rep. CMU-RI-TR-92-01, 1992.',
    'B. Sinopoli, L. Schenato, M. Franceschetti, K. Poolla, M. I. Jordan, and S. S. Sastry, “Kalman filtering with intermittent observations,” *IEEE Trans. Autom. Control*, vol. 49, no. 9, pp. 1453–1464, Sep. 2004.',
    'M. Huang and S. Dey, “Stability of Kalman filtering with Markovian packet losses,” *Automatica*, vol. 43, no. 4, pp. 598–607, 2007.',
    'S. Kluge, K. Reif, and M. Brokate, “Stochastic stability of the extended Kalman filter with intermittent observations,” *IEEE Trans. Autom. Control*, vol. 55, no. 2, pp. 514–518, Feb. 2010.',
    'R. Carbonell, Á. Cuenca, J. J. Salt, E. Aranda-Escolástico, and V. Casanova, “Remote path-following control for a holonomic Mecanum-wheeled robot in a resource-efficient networked control system,” *ISA Trans.*, 2024, doi: 10.1016/j.isatra.2024.05.041.',
    'E. Ferrari, A. Morato, F. Tramarin, C. Zunino, and M. Bertocco, “Quantifying the trajectory tracking accuracy in UGVs: The role of traffic scheduling in Wi-Fi-enabled time-sensitive networking,” *Sensors*, vol. 26, no. 3, Art. no. 881, 2026.',
    'Y. Liu, R. Candell, and N. Moayeri, “Effects of wireless packet loss in industrial process control systems,” *ISA Trans.*, vol. 68, 2017, doi: 10.1016/j.isatra.2017.02.005.',
    'N. El-Sheimy, H. Hou, and X. Niu, “Analysis and modeling of inertial sensors using Allan variance,” *IEEE Trans. Instrum. Meas.*, vol. 57, no. 1, pp. 140–149, Jan. 2008.',
    'E. N. Gilbert, “Capacity of a burst-noise channel,” *Bell Syst. Tech. J.*, vol. 39, no. 5, pp. 1253–1265, Sep. 1960.',
    'Wokwi, “Wokwi: Online ESP32, STM32 and Arduino simulator.” [Online]. Available: https://wokwi.com',
    '“Consistency-gated slowdown: simulator, data, ESP32 firmware and demonstration.” [Online]. Available: https://github.com/pushkar1887/co_res (demo: https://pushkar1887.github.io/co_res/)',
    'B. Efron and R. J. Tibshirani, *An Introduction to the Bootstrap*. New York, NY, USA: Chapman & Hall, 1993.',
    'S. Holm, “A simple sequentially rejective multiple test procedure,” *Scand. J. Statist.*, vol. 6, no. 2, pp. 65–70, 1979.',
    'D. J. Schuirmann, “A comparison of the two one-sided tests procedure and the power approach for assessing the equivalence of average bioavailability,” *J. Pharmacokinet. Biopharm.*, vol. 15, no. 6, pp. 657–680, 1987.',
    'C. J. Clopper and E. S. Pearson, “The use of confidence or fiducial limits illustrated in the case of the binomial,” *Biometrika*, vol. 26, no. 4, pp. 404–413, 1934.',
    'E. S. Page, “Continuous inspection schemes,” *Biometrika*, vol. 41, no. 1/2, pp. 100–115, 1954.',
]
B.append(para('<w:keepNext/><w:pageBreakBefore/><w:spacing w:after="240"/><w:jc w:val="center"/><w:outlineLvl w:val="0"/>', runs('REFERENCES', sz=32, bold=True, color=NAVY)))
for k, ref in enumerate(REFS, 1):
    B.append(para('<w:spacing w:after="80" w:line="300" w:lineRule="auto"/><w:ind w:left="576" w:hanging="576"/><w:jc w:val="both"/>',
                  runs(f'[{k}]', sz=22) + '<w:r><w:tab/></w:r>' + runs(ref, sz=22)))

# =========================================================================== assemble the document
def build():
    os.makedirs(os.path.dirname(OUT_DOCX), exist_ok=True)
    zin = zipfile.ZipFile(TEMPLATE)
    doc = zin.read('word/document.xml').decode('utf8')
    styles = zin.read('word/styles.xml').decode('utf8')
    head, rest = doc.split('<w:body>', 1)
    body, tail = rest.rsplit('</w:body>', 1)
    spans = [m.span() for m in re.finditer(r'<w:p[ >].*?</w:p>|<w:tbl>.*?</w:tbl>', body, re.S)]
    blocks = [body[a:b] for a, b in spans]
    text = lambda x: ''.join(re.findall(r'<w:t[^>]*>([^<]*)</w:t>', x))
    find = lambda t, start=0: next(i for i in range(start, len(blocks)) if text(blocks[i]).strip() == t)

    def outline(i, lvl=0):                               # add an outline level (for the TOC field) to an existing heading
        b = blocks[i]
        if '<w:outlineLvl' in b:
            return
        m = re.search(r'<w:pPr>(.*?)</w:pPr>', b, re.S)
        inner = m.group(1)
        new = inner.replace('<w:rPr>', f'<w:outlineLvl w:val="{lvl}"/><w:rPr>', 1) if '<w:rPr>' in inner else inner + f'<w:outlineLvl w:val="{lvl}"/>'
        blocks[i] = b[:m.start(1)] + new + b[m.end(1):]

    i_abs, i_toc, i_lof, i_lot = find('ABSTRACT'), find('TABLE OF CONTENTS'), find('LIST OF FIGURES'), find('LIST OF TABLES')
    i_ch1 = find('CHAPTER 1')
    i_app = find('APPENDIX A: GROUP CONTRIBUTION')
    for t in ['DECLARATION', 'CERTIFICATE', 'ACKNOWLEDGEMENT', 'ABSTRACT', 'TABLE OF CONTENTS', 'LIST OF FIGURES', 'LIST OF TABLES', 'APPENDIX A: GROUP CONTRIBUTION']:
        outline(find(t))

    marker = lambda m: para('<w:spacing w:after="0"/>', runs(m))
    abstract = []
    for lead, txt in ABSTRACT:
        abstract.append(para('<w:spacing w:after="160"/><w:jc w:val="both"/>', runs(lead, bold=True) + runs(txt)))
    abstract.append(para('<w:spacing w:after="160"/><w:jc w:val="both"/>', runs('Keywords: ', bold=True) + runs(KEYWORDS)))

    # replacement plan (indices refer to the template's top-level blocks)
    plan = {}
    plan[(i_abs + 1, find('', i_abs + 1) if False else i_abs + 7)] = ''.join(abstract)          # 6 placeholder paragraphs
    plan[(i_toc + 1, i_toc + 3)] = marker('[[TOC]]')                                             # tip + manual table
    plan[(i_lof + 1, i_lof + 2)] = marker('[[LOF]]')                                             # manual table
    plan[(i_lot + 1, i_lot + 2)] = marker('[[LOT]]')
    i_pb_app = i_app - 1                                                                         # page-break paragraph before the appendix
    assert 'w:type="page"' in blocks[i_pb_app]
    plan[(i_ch1, i_pb_app)] = ''.join(B)
    assert text(blocks[i_abs + 6]).startswith('Keywords'), text(blocks[i_abs + 6])
    assert blocks[i_toc + 2].startswith('<w:tbl>') and blocks[i_lof + 1].startswith('<w:tbl>') and blocks[i_lot + 1].startswith('<w:tbl>')

    out, pos = [], 0
    for (a, b), rep in sorted(plan.items()):
        out.append(body[pos:spans[a][0]])
        # keep untouched blocks between replacements (with any outline edits)
        out.append(rep)
        pos = spans[b - 1][1]
    out.append(body[pos:])
    new_body = ''.join(out)
    # re-apply outline edits made to untouched blocks
    for i, (a, b) in enumerate(spans):
        orig = body[a:b]
        if orig != blocks[i] and orig in new_body:
            new_body = new_body.replace(orig, blocks[i], 1)

    cap = lambda sid: (f'<w:style w:type="paragraph" w:customStyle="1" w:styleId="{sid}"><w:name w:val="{sid}"/><w:basedOn w:val="Normal"/>'
                       '<w:qFormat/><w:pPr><w:jc w:val="center"/></w:pPr><w:rPr><w:b/><w:sz w:val="21"/></w:rPr></w:style>')
    toc = lambda sid, name, ind: (f'<w:style w:type="paragraph" w:styleId="{sid}"><w:name w:val="{name}"/><w:basedOn w:val="Normal"/><w:next w:val="Normal"/>'
                                  f'<w:uiPriority w:val="39"/><w:unhideWhenUsed/><w:pPr><w:tabs><w:tab w:val="right" w:leader="dot" w:pos="8659"/></w:tabs>'
                                  f'<w:spacing w:after="60" w:line="300" w:lineRule="auto"/><w:ind w:left="{ind}"/></w:pPr></w:style>')
    styles = styles.replace('</w:styles>', cap('FigCap') + cap('TabCap') + toc('TOC1', 'toc 1', 0) + toc('TOC2', 'toc 2', 360) + '</w:styles>')

    zout = zipfile.ZipFile(OUT_DOCX, 'w', zipfile.ZIP_DEFLATED)
    for item in zin.infolist():
        data = zin.read(item.filename)
        if item.filename == 'word/document.xml':
            data = (head + '<w:body>' + new_body + '</w:body>' + tail).encode('utf8')
        elif item.filename == 'word/header1.xml':
            data = data.decode('utf8').replace('>[PROJECT TITLE IN <', '>Consistency-Gated Speed Control<').replace('>SHORT]  |<', '>' + chr(160) * 2 + '|' + chr(160) * 2 + '<').encode('utf8')
        elif item.filename == 'word/styles.xml':
            data = styles.encode('utf8')
        zout.writestr(item, data)
    zout.close()
    print('xml done:', len(B), 'blocks')


def finish_in_word():
    import win32com.client
    word = win32com.client.DispatchEx('Word.Application')
    word.Visible = False
    word.DisplayAlerts = 0
    doc = word.Documents.Open(OUT_DOCX)
    try:
        todo = []
        for p in doc.Paragraphs:
            t = p.Range.Text
            if '[[' in t:
                todo.append((p, t))
        for p, t in todo:
            r = p.Range
            if t.startswith('[[IMG|'):
                path, width = t.strip()[6:-2].split('|')
                rr = doc.Range(r.Start, r.End - 1); rr.Text = ''
                shp = doc.InlineShapes.AddPicture(path, False, True, rr)
                shp.LockAspectRatio = -1
                shp.Width = float(width) * 72
            elif '[[EQ:' in t:
                s, e = t.index('[[EQ:'), t.rindex(']]') + 2
                lin = t[s + 5:e - 2]
                rr = doc.Range(r.Start + s, r.Start + e)
                rr.Text = lin
                om = doc.OMaths.Add(rr)
                om.OMaths(1).BuildUp()
            elif t.startswith('[[TOC]]') or t.startswith('[[LOF]]') or t.startswith('[[LOT]]'):
                code = {'[[TOC]]': 'TOC \\o "1-2" \\h \\z \\u', '[[LOF]]': 'TOC \\h \\z \\t "FigCap,1"', '[[LOT]]': 'TOC \\h \\z \\t "TabCap,1"'}[t[:7]]
                rr = doc.Range(r.Start, r.End - 1); rr.Text = ''
                doc.Fields.Add(rr, -1, code, False)
        for _ in range(2):
            doc.Repaginate()
            doc.Fields.Update()
        doc.SaveAs2(OUT_DOCX, 16)
        doc.ExportAsFixedFormat(OUT_PDF, 17)
        print('pages:', doc.ComputeStatistics(2), '| words:', doc.ComputeStatistics(0))
    finally:
        doc.Close(False)
        word.Quit()


if __name__ == '__main__':
    build()
    finish_in_word()
