# Discrete balance run b6a33b4d007f4e2d9b77384cf29f9619

Snapshot inspected through logged training update 2080, about 5.35 hours after
the run began. MLflow still reported `RUNNING`; these are observations at that
point, not a final result. Parent run: `9cf88add1adc403b83f8fb7231b612c9`.

## Configuration and learning progress

- CPU, 16 training environments, one 64-unit GRU layer, 32-step TBPTT.
- Balance curriculum stage; wheel-speed difference and both head velocities
  forced to zero. Corresponding reward components were zero in inspected CSVs.
- Wheel acceleration used 17 categorical bins spanning **−150 to +150 rad/s²**.
- Adam learning rate 0.0005, discount 0.99, gradient clip 10.
- Training initial conditions were unrandomized; validation used 20 fixed-seed
  randomized episodes, seed 1042.

| Metric | Initial | Latest inspected |
|--------|---------|------------------|
| Mean training episode duration | 0.408 s | 3.563 s at update 2080 |
| Mean validation episode duration | 0.419 s | 2.972 s at update 2000 |
| Mean validation return | −5.950 | 66.466 at update 2000 |
| Validation survival to 10 seconds | 0% | 0% |
| Validation forward-speed MAE | 0.118 m/s | 0.140 m/s |

The best logged training-batch mean duration was 5.362 s at update 1990.
None of the 21 inspected validation evaluations had a survivor at the 10-second
gate. Thus the policy is learning, with substantial late improvement, but is
still below both the 90% survival and 0.05 m/s error requirements.

## Recorded actions

The evaluation CSV at update 2000 lasted 5.19 s (519 actions):

- −10 rad/s²: 353 actions (68.0%).
- +75 rad/s²: 47 actions (9.1%).
- An extreme ±150 command: one action (0.19%).
- No inferred wheel-velocity target saturation at ±10 rad/s.

The update-1800 CSV lasted 5.65 s and contained no ±150 commands, yet still fell.
Rare extreme-bin samples alone therefore do not explain these failures. Broader
sampling around a strongly preferred action, and inadequate closed-loop regulation,
remain relevant.

All 209 logged combined gradient norms exceeded the clip threshold: minimum
24.17, maximum 9008.60. These are pre-clipping norms and show clipping was active.
This is a separate optimization observation; these combined
norms do not identify how much came from the actor versus the critic.

## Same-checkpoint sampled versus deterministic evaluation

Downloaded `checkpoints/best.pt`, whose next iteration was 2000, and evaluated it
on the same 20 validation cases. The sampled result reproduced the logged
benchmark. Deterministic evaluation selected each categorical head's argmax;
curriculum-inactive heads remained zero in both cases.

| Metric | Sampled actions | Deterministic argmax |
|--------|-----------------|----------------------|
| Mean episode duration | 2.972 s | 8.443 s |
| Survival to 10 seconds | 0% | 50% |
| Forward-speed MAE | 0.1396 m/s | 0.2648 m/s |
| Mean return | 66.466 | −128.417 |

Removing sampling improved upright survival substantially, supporting the concern
about exploration noise. It also increased drift and reduced return. Simply
deploying argmax is therefore not a solution to the balance-and-stop task.
Argmax changes the average control as well as removing randomness, so this
comparison does not isolate noise from action bias.

## Next comparison

`config/continuous.yaml` provides tanh-Gaussian heads with the same physical action
limits, local initial exploration, and a learned bounded standard deviation per
head. Keep rewards, curriculum gates, discount and optimizer settings fixed for
the comparison. This is an experiment, not evidence that continuous actions alone
will solve the task.

Paired validation logs sampled and deterministic behavior separately; curriculum
promotion and rollback continue to use the sampled benchmark. Initial versus
learned standard deviations are also logged. If local exploration still fails,
inspect loss-gradient contributions, reward tradeoffs and actuator dynamics before
making several further changes at once.
