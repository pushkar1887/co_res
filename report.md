# Report plan (B.Tech Project-I report)

Planning file for the Word report. The report itself is `report/BTech_Report.docx`, built by `report/build_report.py` from the department template.

## 1. Title

**Consistency-Gated Speed Control for Warehouse Robots During Camera Blackouts: Analysis, Simulation and Embedded Validation**

- Cover (all caps): CONSISTENCY-GATED SPEED CONTROL FOR WAREHOUSE ROBOTS DURING CAMERA BLACKOUTS
- Why this title: it names the method (consistency-gated speed control), the setting (warehouse robots, camera blackouts) and the three kinds of evidence (analysis, simulation, embedded port).
- Rejected: "Slow Down Only When Sensors Disagree" is a paper hook, too informal for a report cover. "Fault-Aware Navigation…" overclaims: we do not do navigation.

## 2. Framing (one paragraph the whole report serves)

A robot that loses its external position fix must choose between speed and safety. Today's practice slows on every loss (safe, but costly). We show that the robot can tell *which* losses are dangerous using two sensors it already carries: the gyro and the wheel encoders both measure turning, and during a blackout their disagreement exposes a fault that nothing else can see. Gating the slowdown on that disagreement keeps the safety of always slowing (−0.19 mm difference) while removing its cost on healthy blackouts (2.6 s saved per 4 s blackout). Closed-form analysis predicts these numbers before the runs. A float32 ESP32 port reproduces the simulator to 0.002 mm.

Writing rules: short declarative sentences; one claim per sentence; every number traced to `results/`; say "simulated" wherever a reader could assume hardware; state limits plainly (Chapter 6).

## 3. Format (from the template's instruction box)

- Times New Roman throughout. Body 12 pt, justified, 1.5 line spacing (template's Normal style).
- Chapter label bold 12 pt (slate); chapter title 16 pt bold navy (1B365D) with a rule; section headings 13 pt bold navy.
- Captions 10.5 pt bold, centred. Figure captions below the figure, table captions above the table. Numbering by chapter (Figure 3.1, Table 5.2).
- Tables: template style (navy header, white bold 10 pt; light grey borders).
- Equations: Word equations, numbered (4.1) on the right. Written in LaTeX in `build_report.py`, because Word's equation input on this machine is set to LaTeX (UnicodeMath strings built up only partially).
- IEEE numbered citations in order of first appearance.
- Untouched: pages 1–3 (cover, declaration, certificate), the acknowledgement placeholders and Appendix A (group contribution). The instruction box on page 2 stays for the student to delete.
- Front matter: abstract (250–350 words, the template's four bold lead-ins + keywords); Table of Contents, List of Figures and List of Tables as Word fields (right-click → Update Field after edits).

## 4. Outline (template chapters kept; sections adapted)

| Chapter | Sections | Figures / tables |
|---|---|---|
| Abstract | Background & Motivation · Methodology · Results & Analysis · Conclusion · Keywords | — |
| 1 Introduction | 1.1 Background · 1.2 Problem Statement · 1.3 Objectives · 1.4 Contributions · 1.5 Scope and Organization | — |
| 2 Literature Review | 2.1 Historical Evolution · 2.2 Classification of Existing Approaches · 2.3 Prior Research and the Gap | T2.1 literature |
| 3 System Design & Methodology | 3.1 Architecture and Working Principle · 3.2 Simulated Robot, Sensors and Faults · 3.3 System Parameters · 3.4 State Estimation · 3.5 Path Tracking · 3.6 Consistency Monitor · 3.7 Speed Policies | F3.1 architecture (new); T3.1 parameters; T3.2 policies |
| 4 Mathematical Modeling & Analysis | 4.1 Governing Equations · 4.2 Drift Under Each Fault · 4.3 Survival Time and Detection Delay · 4.4 Cost and Regret of Each Policy · 4.5 Threshold Design Window · 4.6 Numerical Case Study | T4.1 drift by fault; T4.2 predicted B0 values; F4.1 threshold window |
| 5 Implementation, Testing & Results | 5.1 Implementation (simulator, analysis, ESP32 firmware, demo) · 5.2 Experimental Protocol · 5.3 Verification · 5.4 Short Gaps (A) · 5.5 Monitor Calibration · 5.6 Main Result (B0) · 5.7 Fault Map (B) · 5.8 Scaling and Regret (C) · 5.9 Burstiness (D) · 5.10 Sensitivity (S) · 5.11 Embedded Parity (V8) · 5.12 Discussion | F5.1 Wokwi circuit; F5.2 viva timeline; F5.3 policy trade-off; F5.4 short gaps; F5.5 fault map; F5.6 scaling; F5.7 detection/regret; F5.8 burstiness; T5.1 experiments; T5.2 verification; T5.3 primary tests; T5.4 predicted vs measured; T5.5 sensitivity; T5.6 V8 parity |
| 6 Advantages, Limitations & Applications | 6.1 Advantages · 6.2 Limitations · 6.3 Applications | — |
| 7 Conclusion & Future Scope | 7.1 Conclusion · 7.2 Future Scope | — |
| References | 20 IEEE entries (18 checked against OpenAlex by DOI or title; plus the Wokwi website and this repository) | — |

## 5. Key numbers (all from `results/`, PLAN §20)

- **B0 primary** (T_b = 4 s, arc entry, +1°/s gyro step, 30 paired seeds, Holm): P1 SA − SB = −0.19 mm [−0.51, +0.19] (non-inferior, δ = 5 mm); P2 2.60 s [2.60, 2.60] saved on healthy blackouts; P3 N − SA = 31.1 mm [28.4, 33.8]; P4 0/120 false alarms (CP upper 2.5%).
- Breaches at 4 s with the fault: N 100% (entry) / 97% (mid); SA, SB, SO 0% (entry) / 10–13% (mid). Alarm delay 0.50–0.54 s (predicted 0.60 s).
- Time costs predicted / measured: SB 2.59 / 2.60 s; SA fault 2.03 / 2.04–2.10 s; SO 2.45 / 2.46 s.
- Formula check (e_dev, mid onset): N 50.5, SA 17.2, SB 16.8, SO 16.9 mm vs 53.5 / 17.1 / 16.0 / 16.0 (1–6%). Entry onset overpredicted 15–34% (curvature).
- Verification: V4 0.001 mm; V5 NEES 1.90; V6 ARW 0.04% error; V7 ≤ 3.9%; V10 0/9,500 truncated.
- A: EKF estimate error 4.5–7.3× and steering jump 3.7–21× smaller than DR; HOLD worst mid-turn 9.7 mm.
- CAL: v1 7/480 false alarms → K2; v2 max |D| 0.318°, 0/480, margin 1.9×.
- B: SA − SB within ±2.5 mm in every fault cell; DR healthy 19–42 mm (breach 30–70%); wheels +3% breaches for every policy.
- C: gyro error ∝ v, R-independent, 0.80–0.85 of ½vbT²; regret within ±1.5 mm; t_d 2.16 … 0.26 s for b = 0.2 … 2°/s.
- D: HOLD 11.6 → 39.9 mm as burst 1 → 16; EKF flat 8.0–8.6 mm.
- S: SA − SB within ±0.7 mm, SA healthy cost 0 s under 2× noise, 50 Hz steering, α ∈ {0.2, 0.5}.
- V8: 0.0024 / 0.0011 mm, 0/706 alarm differences, 38–127 µs per step.

## 6. Build

1. `report/make_figures.py`: architecture diagram (Figure 3.1, matplotlib); Wokwi circuit screenshot saved to `report/fig_wokwi.png`.
2. `report/build_report.py`: copies the template; replaces the abstract, front-matter lists and Chapters 1–7 + references in `word/document.xml` with paragraphs cloned from the template's own formatting; adds outline levels and caption styles; then Word (COM) inserts figures, builds equations, inserts the TOC/LoF/LoT fields, updates them and exports a PDF for checking.
3. Check every page of the PDF (rendered with PyMuPDF): no blank pages, no leftover `[[` markers or raw LaTeX.

## 7. Status (2026-11-23)

- Built: `report/BTech_Report.docx` and `.pdf`, 39 pages (front matter i–viii, Chapters 1–7 and references 1–30, Appendix A 31). The Word and PDF files are git-ignored: the student adds personal details to them, and the template is the department's.
- Left for the student: cover, declaration, certificate, acknowledgement, Appendix A, deleting the instruction box on page ii, and right-click → Update Field on the contents and lists after any edit.
