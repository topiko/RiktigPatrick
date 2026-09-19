# Run 9d53d52c: recovery stopping and exploding recurrent gradients

Run `9d53d52c67364fc2855f1ea2969750c1` used TBPTT **256**, acceleration initial std
**0.025**, gamma **0.995**, 50 validation episodes and a guard activation threshold
of **1000**. The initial rollback required a drop exceeding both 30% and 100 return
units. This analysis did not modify its checkpoints or training settings.

## What stopped the run

| Validation iteration | Return | Mean duration | Decision |
|---|---:|---:|---|
| 2200 | 624.15 | 11.44 s | Best; below guard activation threshold |
| 2750 | −459.52 | 16.64 s | Guard still inactive; position MAE 1.23 m |
| 2760 | 1184.68 | 15.98 s | New best; guard becomes eligible to roll back |
| 2770 | 1185.33 | 15.38 s | New best |
| 2780 | 105.08 | 2.42 s | Material degradation; rollback |
| 2781 | 1171.17 | 15.61 s | Recovery rejection |
| 2782 | 1177.07 | 15.33 s | Recovery rejection |
| 2783 | 1170.11 | 15.55 s | Third rejection; clean stop |

The first rollback followed genuine degradation, not merely crossing 1000. The
subsequent candidates lost only **1.19%, 0.70% and 1.28%** relative to the best.
Strict best-score improvement was still required on every recovery attempt. This
repeats the overly strict stopping behavior observed in `1c81aacb`.

The final checkpoint's weights exactly match the saved best policy at iteration
2770. The best score had episode-return std **24.26** over 50 episodes, so this
result was much more consistent than earlier outlier-dominated benchmarks. It
still did not meet the curriculum goal: sampled 20-second survival was zero,
position MAE **0.183 m** and velocity MAE **0.099 m/s**.

## New finding: gradient amplification

The logged gradient norm is measured **before** clipping. It reached approximately
**6.31e17** at update 1240; values around **1e16** occurred near the final best
policy. These were finite gradients, so numerical recovery did not trigger.

To isolate the cause, the iteration-2770 policy was frozen and two matched
16-episode batches were collected at seeds 20000 and 20016. The only changed
setting was the detach period. All observations, actions, rewards and forward
policy outputs were verified bitwise identical across periods. No optimizer step
was applied in this experiment.

| TBPTT steps | Pre-clipping gradient norm, batch 1 | Batch 2 |
|---|---:|---:|
| 32 | 2.43e4 | 2.22e4 |
| 64 | 1.45e6 | 1.14e6 |
| 128 | 5.28e9 | 2.24e9 |
| 256 | 3.97e16 | 1.85e15 |

The amplification is concentrated in the input encoders and GRU, and occurs in
both actor and critic gradients. In batch 1, the action-head gradient norm stayed
at **210.82** and the value-head norm at **354.43** across all four periods. At
256 steps, global clipping to 500 multiplies every gradient by approximately
**1.26e−14**, almost eliminating those heads' current gradients before Adam.
Previously accumulated optimizer moments can still move their parameters.

Thus the longer backpropagation window exposes severe recurrent gradient
amplification in this trained policy. Clipping bounds the total gradient norm but
does not repair its direction or the extreme imbalance between parameter groups.
Reducing the clipping threshold alone does not address that imbalance.

This is different from the earlier TBPTT experiment on the older iteration-570
policy: that policy did not exhibit remotely comparable amplification. The result
is checkpoint-dependent, not a claim that 256 steps always fail.

## Recommended next experiments

1. Recovery now uses the original degradation threshold rather than requiring a
   new record. For this run's best return and drop limits, the floor is **829.73**;
   each of the three logged recovery candidates would pass that acceptance check.
   Continued training would then follow a different trajectory because accepted
   candidates are retained.
2. Test a shorter window, initially 32, from the preserved good checkpoint while
   measuring gradient norms and actual validation changes. Even 32 still clips;
   the gradient experiment alone does not establish that resumed learning improves.
3. Keep new-best videos, plots and CSV traces so useful policies remain observable
   even when their improvements occur between periodic artifact intervals.

Scratch results are in `/tmp/opencode/rp_9d53_run.json` and
`/tmp/opencode/rp_9d53_gradients.json`; experiment script:
`/tmp/opencode/gradient_9d53.py`.
