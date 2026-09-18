# Repeated decay after rollback: September 18, 2026

Checkpoint: `outputs/2026-09-18/13-26-18/checkpoints/best.pt`, iteration **570**.
Continuous wheel acceleration, position holding, wheel-speed cap 20 rad/s,
acceleration limit 150 rad/s², critic coefficient 0.1, discount 0.99, 10 ms control
period and TBPTT every 32 decisions. All experiments ran on CPU, with one Torch
thread, and left the checkpoint unchanged.

## Reproduction and single-batch controls

The original fixed-suite score reproduced exactly: **332.8465887073092**; mean
duration 5.373 s. With one common rollout batch, restored Adam, and LR 0.00018:

| Candidate | Validation return |
|---|---:|
| Unchanged checkpoint | 332.85 |
| Normal combined actor/critic update | 270.96 |
| Critic trains its head only; actor still trains backbone | 286.75 |
| Normal update, fresh Adam | 261.65 |
| Normal update, LR reduced tenfold | 327.52 |

The weighted critic shared-gradient norm was **183.44**, versus the actor's
**18.08**. The normal update's conditional policy KL was only **0.00140**. Thus
large return degradation need not involve a large measured conditional KL.
This probe does not replay the updated recurrent history.

Fitting the critic head to the batch while keeping the policy fixed reduced MSE
from 859.45 to 34.33. An actor-only update afterward still reduced return to 292.12
with restored Adam. These observations do not establish critic interference or
optimizer momentum as the sole cause.

## Independent actor-gradient batches

Command template:

```bash
uv run python -m sim.diagnose_updates \
  outputs/2026-09-18/13-26-18/checkpoints/best.pt \
  --batches 4 --learning-rate 0.00018 --output /tmp/actor_batches.json
```

Four batches of 16 episodes used seeds 20000, 20016, 20032 and 20048. Every candidate
started from the same weights and saved Adam moments. Only actor gradients were
applied. Each validation suite contained ten episodes, with seed 1042 or 101042.

| Candidate / gradient objective | Original suite | Second suite |
|---|---:|---:|
| Unchanged checkpoint | 332.85 | 350.72 |
| Batch 0, original objective | 295.55 | 315.72 |
| Batch 1, original objective | 292.57 | 312.71 |
| Batch 2, original objective | 301.46 | 321.24 |
| Batch 3, original objective | 290.73 | 309.38 |
| Mean of four batches, original objective | 295.40 | 315.30 |
| Mean, zero speed penalty during collection | 294.87 | 314.43 |
| Mean, discount 0.999 during collection | 302.47 | 323.10 |

Actor-gradient cosine similarities ranged from approximately **−0.834 to +0.786**:
the raw batch gradients disagree substantially. However, averaging these four
batches did not yield an improving Adam update. Restored shared-parameter moments
include past critic gradients, so actor-only gradients do not imply actor-only
optimizer history.

Reward/discount ablations used the same policy, sampling seeds and physical
parameters for collection. Evaluation always retained the original reward. The
speed-penalty ablation did not improve these one-step results. A longer discount
horizon helped relative to the original update, but still lost to doing nothing.
The old critic was retained in both cases; these are not full retraining studies.

## Is speed damping preventing recovery?

Physically, moving the wheels further in the direction of a fall can be necessary
to get the contact point under or beyond the centre of mass before braking.
A speed penalty can oppose this short-term movement. In the hold stage the actual
penalty is `-0.5 * abs(v)`, not the full `-4 * abs(v)` used in locomotion. At
0.5 m/s it costs 0.25 per nominal step against a survival reward of 1, before
position and pitch penalties.

With gamma 0.99 at 100 Hz, rewards one second ahead have weight about 0.366 and
five seconds ahead about 0.0066. Gamma 0.999 extends the approximate discount
horizon from one second to ten. This is separate from TBPTT's 0.32-second gradient
window: return targets still include later rewards. The physical hypothesis is
plausible, but removing speed damping did not fix this checkpoint's update direction.

## Acceleration exploration noise

The learned pre-tanh std was **0.09931**, close to its 0.1 initialization. Near a
zero mean, `150 * std` gives approximately **14.90 rad/s²** of acceleration noise,
or **0.149 rad/s** in the next nominal wheel-speed target. This is a local
approximation; tanh compresses the distribution away from zero.

Only the acceleration std was changed for these evaluations; all mean-policy
weights and reward settings were preserved. Steering/head actions were inactive.

| Pre-tanh std | Original return | Second return | Original duration | Original velocity MAE |
|---|---:|---:|---:|---:|
| Learned 0.09931 | 332.85 | 350.72 | 5.373 s | 0.214 m/s |
| 0.05 | 346.28 | 364.36 | 5.541 s | 0.233 m/s |
| 0.025 | 349.09 | 367.83 | 5.603 s | 0.242 m/s |
| Deterministic | 348.17 | 369.26 | 5.639 s | 0.250 m/s |

Lower noise modestly improves return and duration, without improving position
holding or velocity error. It does not explain the much larger update-driven
degradation. A 0.05 initial std is a reasonable separate training experiment;
evaluation alone does not establish a better training exploration schedule.

Reducing only the critic-head LR would slow the value fit without directly limiting
the critic gradient entering the shared GRU. A critic-only LR and shared-gradient
control are different interventions. No critic-LR, reward, discount or exploration
default was changed based on these one-step experiments.

## Bounded recovery verification

A short resumed CLI run used deliberately sensitive rollback thresholds to enter
recovery immediately. The first rejected update scored 269.26; recovery attempts
scored 293.81, 281.59 and 278.93. LR stayed at **0.000171** throughout those attempts.
Training stopped at next iteration **574** with the best policy restored and a
`training_stop.json` report. Unit tests additionally verify immediate checks between
normal scheduled evaluations, full Adam restoration, seed progress and clean resume.

The bounded guard prevents an endless rollback/LR-decay cycle. It does not make an
unproductive gradient estimator learn; the measured learning problem remains open.
