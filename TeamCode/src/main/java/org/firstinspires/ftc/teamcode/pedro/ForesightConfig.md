# Pedro Pathing 3.0: `ForesightConfig` reference

Every field of `com.pedropathing.algorithm.ForesightConfig` (Pedro core 3.0.1), what it does,
its units and default, and what we set in `Constants.java`.

**Sources.** Defaults and behaviour come from reading the compiled library (`core-3.0.1.jar`),
because the official docs only cover about half of these fields:
[Foresight Options](https://pedropathing.com/docs/pathing/reference/foresightoptions) and
[Path Constraints](https://pedropathing.com/docs/pathing/reference/pathconstraints).
Where the code and the docs disagree, it's noted. Re-check after upgrading Pedro.

**Conventions.** Distances are inches, speeds in/s, accelerations in/s², angles radians,
times milliseconds. `NONE` (`ForesightConfig.Constraint.NONE`) means "no limit".
**Required** fields have no default: the config fails if they aren't set.

## How a path runs (enough to make sense of the fields)

Each loop, Foresight:

1. Finds the closest point on the path (`t` from 0 to 1) and a **projected** point: where the
   robot would stop if it braked now (from the brake coefficients).
2. **Cruise ("coast")** while that projected stop point is still short of the path end: the
   target speed is the robot's top speed, cut down by the speed limits, and the `coast`
   controller turns it into drive power.
3. **Brake** once the projected stop point reaches the end (if `brakeAtEnd`): the target speed
   becomes "the speed you could still stop from in the distance left", and the `brake`
   controller turns it into power. Reverse power is limited by `maxBrakingPower`.
4. Adds translational correction (`forwardTranslational` / `strafeTranslational`) and heading
   correction (`headingFeedback` + `headingStaticFF`), then shares the motor power out by
   priority (see `headingDriveRatio`).
5. Once `t` reaches `1 - parametricTConstraint`, it **holds** the end point until the end
   conditions are met (`*Constraint` fields below), then the path is done.

## Controllers (all required except `headingStaticFF`)

| Field | Default | What it does | Ours |
|---|---|---|---|
| `forwardTranslational` | required | Position error along the robot's forward axis (robot frame, in) → forward power. Called per axis, so a `piecewise` switch applies to the forward error alone. | Piecewise: secondary P (+ end integral) under 2.5 in, primary P above |
| `strafeTranslational` | required | Same for the strafe axis. | Same, strafe gains |
| `headingFeedback` | required | Heading error (rad) → turn power. | P 2.59 (proto), 2.0 (prod) |
| `headingStaticFF` | `staticFeedforward(0)` | Fixed turn power added in the direction of the heading error, to overcome friction on small heading errors. | default (0) |
| `coast` | required | Target speed → drive power while cruising. Tuned by AutoTune. | `proportionalFeedforward` |
| `brake` | required | Target speed → drive power while braking at the end. Tuned by AutoTune. | `proportionalFeedforward` |

## Robot model (all required; from AutoTune)

| Field | Units | What it does |
|---|---|---|
| `maxAchievableForwardVelocity` | in/s | Top speed driving forward. Interpolated with strafe by direction of travel to get the top speed on each path. **Use the measured value**: it is the base the speed limits scale from and what Foresight predicts with. |
| `maxAchievableStrafeVelocity` | in/s | Top speed strafing. |
| `naturalForwardDeceleration` | in/s² | How fast the robot slows forward with zero power. Used by `maxDecelerationScale` and coast-to-stop predictions. |
| `naturalStrafeDeceleration` | in/s² | Same for strafing. |
| `linearBrakeCoefficients` | diag(fwd, strafe) | Stopping distance model: distance = linear·v + quadratic·v², per axis. Used to predict where the robot will stop (when to start braking) and the speed to brake from. |
| `quadraticBrakeCoefficients` | diag(fwd, strafe) | The v² term of the same model. |
| `headingBrakeCoefficients` | (linear, quadratic) | Same idea for how far the heading keeps turning while braking. |

## Speed limits while cruising

Applied in this order; the lowest wins.

| Field | Default | Units | What it does |
|---|---|---|---|
| `maxPathSpeed` | NONE | fraction 0-1 | Target speed ≤ `maxPathSpeed` × top speed for this direction. **Not in/s**: a value like 20 caps nothing. |
| `maxVelocityConstraint` | NONE | in/s | Hard cap on target speed. **This is what our `AUTO_MAX_SPEED` sets.** |
| `maxAccelerationConstraint` | NONE | in/s² | Target speed may rise by at most this × loop time per loop (soft start). |
| `maxDecelerationConstraint` | NONE | in/s² | If set, slows the cruise smoothly toward the end: target ≤ √(`coastDownToVelocity`² + 2·decel·distance left). Braking still finishes the stop. |
| `maxDecelerationScale` | NONE | fraction | Sets the deceleration above to this × natural deceleration (lower of the two if both set). |
| `coastDownToVelocity` | 0 | in/s | The speed the deceleration limit slows to, instead of 0. Only matters if a deceleration limit is set. |

## Braking at the end

| Field | Default | Units | What it does | Ours |
|---|---|---|---|---|
| `brakeAtEnd` | true | bool | Brake at the path end. False keeps cruising through the end point (the hold then catches it). | default |
| `maxBrakingPower` | 0.2 | power | Caps drive power that **opposes** the current motion (per axis). Power in the direction of motion isn't limited. The docs warn high values can overload the Control Hub. | default |
| `brakeAggression` | 1.0 | ratio | Divides the distance left when working out the speed to brake from. The docs say > 1 allows overshoot; the code divides the distance by it, which reads as > 1 brakes earlier (undershoot). **Unverified: test on the robot.** | default |
| `pathSkip` | true | bool | In a chain, when braking would start and more paths follow, skip straight to the next path instead of braking between them. | default |

## Correction priority

| Field | Default | Units | What it does |
|---|---|---|---|
| `headingDriveRatio` | 0.5 | 0-1 | Share of the heading correction that gets top priority for motor power. Power is given out in order (curve feedforward, heading feedforward + this share, then translational / rest of heading / drive in an order that depends on the tolerances below). Only matters when the motors are saturated (fast driving). Higher = holds heading tighter at speed. |
| `translationalDeviationTolerance` | 2.5 | in | Position error above which translational correction is put ahead of driving forward. Also used by `cosineScale`. |
| `headingDeviationTolerance` | 11.25° (0.196 rad) | rad | Heading error above which heading correction moves ahead of translational and drive. Also used by `cosineScale`. |
| `cosineScale` | false | bool | If on, forward drive power drops to 0 while the position error is over 2 × `translationalDeviationTolerance` or the heading error over 2 × `headingDeviationTolerance`, so the robot fixes the error before driving on. Also slows the start of a path if the robot starts off it. |
| `minCorrectionDistance` | 0.001 | in | Dead band: position errors smaller than this get no translational correction (the controllers aren't called, so an integral term pauses). Undocumented. |

## Holding the end point and finishing

| Field | Default | Units | What it does | Ours |
|---|---|---|---|---|
| `parametricTConstraint` | 0.025 | path fraction | The path counts as at its end once `t ≥ 1 − this` (97.5 %); from then the robot holds the end point. Also marks the first 2.5 % as the path start. | default |
| `holdPointTranslationalScaling` | 0.45 | × | Multiplies translational correction while holding. | **1.0** proto (0.45 was too weak to move the last couple of inches); default on prod |
| `holdPointHeadingScaling` | 0.35 | × | Multiplies heading correction while holding. | default |
| `translationalConstraint` | 0.1 | in | Done when the distance to the end point is under this… | default |
| `headingConstraint` | 0.007 (0.4°) | rad | …and the heading error is under this… | default |
| `velocityConstraint` | 0.1 | in/s | …and the speed toward the end point is under this. | default |
| `timeoutConstraint` | 100 | ms | …or this long has passed since holding started, whichever comes first. The path (and its `FollowPathCommand`) then finishes. | **500** proto; default on prod |

## Notes for this repo

- Production doesn't set `holdPointTranslationalScaling` or `timeoutConstraint`, so it uses the
  defaults (0.45, 100 ms). Copy the prototype's values if production stops short or hands off
  to the next path too early.
- Our end integral (`Constants.endIntegral`) lives inside the translational controllers; see the
  comment there. It only works when the error is above `minCorrectionDistance` and the hold
  scaling is applied to its output too.
- Pedro calls `reset()` on all these controllers when a new path starts (`Follower.follow`).
