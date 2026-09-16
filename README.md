# SIXT33N — Voice-Controlled Car

A voice-controlled car built for UC Berkeley's **EECS 16B: Designing Information Devices and Systems II** (Spring 2024). SIXT33N drives itself using an onboard microphone, a two-motor drivetrain, and an Arduino — no remote control, no external computer in the loop. It listens for one of four spoken commands, classifies the word using a lightweight PCA-based classifier, and executes the corresponding maneuver using closed-loop motor control tuned from an experimentally identified motor model.

Built by **Beia Cabrera Sanchez** and **Cassandra Dove**.

## Demo

[`media/demo_final_run.mp4`](media/demo_final_run.mp4) — the final integration demo: the car listening for a voice command and driving the corresponding maneuver.

## How it works

The project was built up in three stages, each addressed in its own lab, before being combined into the final integrated system.

### 1. System identification (`lab06_system_identification/`)

Before the car can be controlled, its motors need to be characterized. We swept a range of PWM duty cycles to each wheel and measured the resulting wheel velocity via onboard encoders (`motor_dynamics_sweep.ino`), then fit a linear model `v = θ·PWM + β` per wheel. Theta (θ) captures how sensitive velocity is to PWM input; beta (β) captures the real-world offset from static friction and wheel imbalance. Our fitted values were θ_left = 0.3811, θ_right = 0.4371, β_left = -16.8, β_right = -9.645 — the asymmetry between the two wheels is exactly why open-loop control isn't enough on its own.

`Lab6_Handout.pdf` is the original EECS 16B lab handout for reference.

### 2. Closed-loop control

Using the identified motor model, we implemented open-loop control (`u[i] = (v* + β) / θ` per wheel) and then closed-loop control that corrects for drift using the difference in distance traveled by each wheel (`δ[i] = d_L[i] - d_R[i]`). Closed-loop control was necessary because open-loop control has no way to compensate for the wheel asymmetry found in system identification, or for real-world disturbances like friction and noise — the car would otherwise drift instead of driving straight. A time-varying `δ_ref[i]` term lets the same control loop execute controlled 90° turns instead of just straight-line driving. Small "jolts" are applied at the start of motion to overcome static friction.

### 3. Voice classification (`lab08_classification/`)

To recognize spoken commands, we recorded ~40 samples each of four words — **"Latte," "Smoothie," "Pancake,"** and **"Bread"** — chosen for their distinct syllable counts and pronunciation patterns, which make them easy to tell apart even with a simple classifier. Each recording is reduced to its amplitude envelope, then compressed with PCA/SVD down to 3 principal components (`BASIS_DIM = 3`) — small enough to fit in the Arduino's limited memory. A new recording is classified by projecting it onto those 3 components and finding the nearest of four centroids (`EUCLIDEAN_THRESHOLD = 0.04`, `LOUDNESS_THRESHOLD = 140` to reject background noise). `audio_classifier.ino` is the firmware; `classification_analysis.html` is the exported analysis notebook used to build the PCA basis and centroids baked into the firmware.

### 4. Integration & final demo (`lab09_integration_final_demo/`)

`integration_final.ino` is the final combined sketch: listen for a word, classify it, then drive the corresponding maneuver using the closed-loop controller from stage 2. `integration_analysis.ipynb` is the accompanying notebook used to validate the integration before the final demo.

## Report

[`report/Lab_Report_Labs6-8_SystemID_Controls_Classification.pdf`](report/Lab_Report_Labs6-8_SystemID_Controls_Classification.pdf) is our full written report, covering system identification, control theory, and classification design decisions in detail.

## Repo structure

```
├── README.md
├── media/
│   └── demo_final_run.mp4                                          # final demo video
├── report/
│   └── Lab_Report_Labs6-8_SystemID_Controls_Classification.pdf     # written report
├── lab06_system_identification/
│   ├── motor_dynamics_sweep.ino                                    # PWM sweep + encoder logging firmware
│   └── Lab6_Handout.pdf                                            # course lab handout (reference)
├── lab08_classification/
│   ├── audio_classifier.ino                                        # PCA-based voice command classifier firmware
│   └── classification_analysis.html                                # PCA/SVD analysis notebook (exported)
└── lab09_integration_final_demo/
    ├── integration_final.ino                                       # final combined listen → classify → drive firmware
    └── integration_analysis.ipynb                                  # integration validation notebook
```

## Course context

EECS 16B: Designing Information Devices and Systems II, UC Berkeley, Spring 2024. `Lab6_Handout.pdf` in this repo is official course material included for reference; all other files are our own work.
