# RES-492 declared control-strategy search record

The jump strategies in `biosyngen/i2/jumps.py` are control hypotheses. They were selected by the small manual searches below, run in the session that produced commit RES-492. The selection criteria were mechanical: a near-vertical takeoff, a landing that returns to quiet stance, and no refused run. No human data was used, and nothing here is evidence of human strategy.

## Engine findings during the search

The first bilateral jumper runs exposed contact-law defects. Each was fixed in the engine with its own falsifier before tuning continued (commit `19497a7`):

| Observation | Cause | Fix |
| --- | --- | --- |
| Summed foot force down to −0.86 W at push-off | Velocity step without the convective term; bilateral projection held a rolling toe with tension | Convective targets; unilateral velocity and position projections |
| Heel penetration after release | Joint-drift position projection pushed an open heel below the floor | Open contacts held at zero gap within the projection |
| Missed impact | Crossing detection required a closing start velocity | Gap-change crossing detection; grazing closures become touchdowns |
| Zero-time impact loop at heel strike | Greedy active set released the heel instead of the toe | Shared complementarity routine with exhaustive normal-set enumeration |
| Refusal at countermovement onset (loaded models) | Enumeration seeded only with residual-slip modes | Both prepared and all-stick seeds per normal set |
| ~3 N·s missing horizontal impulse | Impact event reported only impacting contacts | All contact and projection impulses of an impact are reported |

## Strategy variables searched

| Variable | Values tried | Outcome | Chosen |
| --- | --- | --- | --- |
| Countermovement reference | Step change | Feet left the floor at onset | 0.40 s minimum-jerk blend |
| Balance (ankle strategy) gain/damping | 3000 / 600, 800, 1200, 1500 | Heel lift at 600; toe lift at 1500 | 3000 / 1200 |
| Propulsion targets (hip, ankle) | Several | No effect: PD saturates every actuator | — |
| Propulsion ankle effort cap | 1.0, 0.75, 0.6, 0.5, 0.45, 0.4 | Takeoff vx −0.47 → −0.07 m/s, H about COM −6.3 → +0.5 | 0.45 |
| Hip/knee effort caps | 0.55–1.0 | Lower hip effort increased backward rotation | 1.0 |
| Landing ankle PD (kp/kd) | 1500/250, 600/120, 300/60, 150/30 | Stiff ankle lifted the toes and rocked the model onto its heels | 150/30 in landing and stand |
| Balance reference frame | World x, mean ankle x | World reference wrong after the feet move | Ankle-relative |
| Hip strategy (sign-corrected) | 0–6000 | Did not rescue the loaded landing | Off by default |
| LOADED_CMJ flight hip angle | 0.10, 0.30, 0.45, 0.60, 0.75 | Only 0.10 landed with the COM over the feet | 0.10 |
| Heavy-jumper strength | Unscaled vs (m/75)(h/1.78) | Unscaled 90 kg collapsed on landing (kept as a refusal control) | Weight × stature scaling |

## Known strategy artifacts

- At the countermovement→propulsion switch, foot force dips briefly (CMJ trace in `jump_family.svg`). This follows from the stage switch and activation dynamics, not from any measured pattern.
- Landing heel strike after toe strike is a rigid impact (19–31 N·s per heel). The ankle cannot absorb a toe-first landing within its declared eccentric capacity. Landing peak force is NOT_EVALUABLE.
- SJ and CMJ takeoff velocities are similar (1.89 vs 1.91 m/s). The usual CMJ advantage is not reproduced: there is no series elasticity, and the activation build-up differs between the strategies.
