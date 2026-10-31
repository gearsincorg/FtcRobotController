# Pedro Pathing 3.0: `ForesightConfig` reference

A field-by-field guide to `com.pedropathing.algorithm.ForesightConfig`, the settings for
Foresight, Pedro Pathing 3's path-following algorithm: what each field does, its units and its
default.

**Version.** Written against Pedro Pathing core **3.0.1**. Defaults may change in later
releases.

**Sources.** The official docs cover about half of these fields:
[Foresight Options](https://pedropathing.com/docs/pathing/reference/foresightoptions) and
[Path Constraints](https://pedropathing.com/docs/pathing/reference/pathconstraints).
Everything else, including all defaults, comes from reading the compiled library. The
descriptions are one reader's interpretation of that code, not official documentation. Where
the code and the docs seem to disagree, it's noted.

**Conventions.** Distances are inches, speeds in/s, accelerations in/s², angles radians, times
milliseconds. `NONE` (`ForesightConfig.Constraint.NONE`) means "no limit". **Required** fields
have no default: you must set them (AutoTune produces most of them).

```java
ForesightConfig foresight = new ForesightConfig(c -> {
    c.headingFeedback.set(Controller.proportional(2.0));
    c.maxVelocityConstraint.set(40.0);   // cap path speed at 40 in/s
    // ...
});
```

## How a path runs

Each loop, Foresight:

1. Finds the closest point on the path (`t` from 0 to 1) and a **projected** point: where the
   robot would stop if it started braking now (from the brake coefficients).
2. **Cruises ("coast")** while that projected stop point is still short of the path end. The
   target speed is the robot's top speed in its direction of travel, reduced by any speed
   limits, and the `coast` controller turns it into drive power.
3. **Brakes** once the projected stop point reaches the end (if `brakeAtEnd` is true). The
   target speed becomes the speed the robot can still stop from in the distance left, and the
   `brake` controller turns that into power. Reverse power is limited by `maxBrakingPower`.
4. Adds translational correction (`forwardTranslational` and `strafeTranslational`) and heading
   correction (`headingFeedback` + `headingStaticFF`), then shares the available motor power
   out by priority (see `headingDriveRatio`).
5. Once `t` reaches `1 - parametricTConstraint`, **holds** the end point until the end
   conditions are met (the `*Constraint` fields at the bottom), and then the path is done.

## Controllers

| Field | Default | What it does |
|---|---|---|
| `forwardTranslational` | required | Position error along the robot's forward axis (robot frame, inches) → forward power. |
| `strafeTranslational` | required | Same for the strafe axis. |
| `headingFeedback` | required | Heading error (rad) → turn power. |
| `headingStaticFF` | `staticFeedforward(0)` | Fixed turn power added in the direction of the heading error. Helps the robot overcome friction on small heading errors. |
| `coast` | required | Target speed → drive power while cruising. Tuned by AutoTune. |
| `brake` | required | Target speed → drive power while braking at the end. Tuned by AutoTune. |

Tips:
- The two translational controllers are called **separately for each axis**, each with only
  its own axis's error. A `Controller.piecewise(...)` switch therefore applies to that axis's
  error, not to the total distance off the path.
- For extra push over the last fraction of an inch, add an integral term to the
  translational controllers. For example, use
  `Controller.proportional(kP).plus(Controller.integral(kI, iZone, decay, maxI))`:
  - it only builds up while |error| < `iZone`;
  - each loop it is multiplied by `decay` (per loop, not per second);
  - it is capped at ±`maxI`, so it adds at most `kI × maxI` power.

  It is not cleared when the error leaves the zone. Pedro calls `reset()` on the controllers
  when a new path starts.

## Robot model (required, from AutoTune)

| Field | Units | What it does |
|---|---|---|
| `maxAchievableForwardVelocity` | in/s | Top speed driving forward. Foresight blends it with the strafe value according to the direction of travel to get the top speed on each part of the path. Use the measured value: the speed limits below scale from it, and Foresight's predictions depend on it. Don't lower it to slow the robot down; use `maxVelocityConstraint` instead. |
| `maxAchievableStrafeVelocity` | in/s | Top speed strafing. |
| `naturalForwardDeceleration` | in/s² | How quickly the robot slows down going forward with zero power. Used by `maxDecelerationScale` and by the coast-to-stop predictions. |
| `naturalStrafeDeceleration` | in/s² | Same for strafing. |
| `linearBrakeCoefficients` | `Matrix.diag(fwd, strafe)` | Stopping-distance model: distance = linear·v + quadratic·v², per axis. Foresight uses it to predict where the robot would stop (which decides when braking starts) and the speed to brake from. |
| `quadraticBrakeCoefficients` | `Matrix.diag(fwd, strafe)` | The v² term of the same model. |
| `headingBrakeCoefficients` | `Vector2D(linear, quadratic)` | The same idea for how far the heading keeps turning while braking. |

## Speed limits while cruising

They are applied in this order, and the lowest result wins.

| Field | Default | Units | What it does |
|---|---|---|---|
| `maxPathSpeed` | NONE | fraction 0-1 | Target speed ≤ `maxPathSpeed` × top speed in the current direction. **This is not in/s**: a value of 20 means 20× top speed, which caps nothing. |
| `maxVelocityConstraint` | NONE | in/s | Hard cap on target speed, the same in every direction. |
| `maxAccelerationConstraint` | NONE | in/s² | Target speed can rise by at most this × loop time each loop, giving a soft start. |
| `maxDecelerationConstraint` | NONE | in/s² | If set, slows the cruise smoothly toward the end: target ≤ √(`coastDownToVelocity`² + 2·decel·distance left). Braking still finishes the stop. |
| `maxDecelerationScale` | NONE | fraction | Sets the deceleration limit to this × the natural deceleration. If both are set, the lower deceleration is used. |
| `coastDownToVelocity` | 0 | in/s | The speed the deceleration limit slows to, instead of 0. Only matters when a deceleration limit is set. |

Pedro 3 has no per-path speed limit. Each Foresight speed limit applies to every path the
follower runs. See
[issue #183](https://github.com/Pedro-Pathing/PedroPathing/issues/183).

## Braking at the end

| Field | Default | Units | What it does |
|---|---|---|---|
| `brakeAtEnd` | true | bool | Brake at the end of the path. If false, the robot cruises through the end point and the hold then has to catch it. |
| `maxBrakingPower` | 0.2 | power | Caps drive power that **opposes** the robot's current motion, per axis. Power in the direction of motion isn't limited. The docs warn that high values can overload the Control Hub. |
| `brakeAggression` | 1.0 | ratio | Divides the distance left when Foresight works out the speed to brake from. The docs say a value above 1 allows overshoot. The code divides the distance by it, which suggests a value above 1 makes the robot brake earlier (undershoot). **Test on your robot.** |
| `pathSkip` | true | bool | In a path chain, once braking would start and more paths follow, move straight on to the next path instead of braking between them. |

## Correction priority

| Field | Default | Units | What it does |
|---|---|---|---|
| `headingDriveRatio` | 0.5 | 0-1 | The share of heading correction that gets top priority for motor power. See below. |
| `translationalDeviationTolerance` | 2.5 | in | Position error above which translational correction is put ahead of driving forward. Also used by `cosineScale`. |
| `headingDeviationTolerance` | 11.25° (0.196 rad) | rad | Heading error above which heading correction moves ahead of translational correction and drive. Also used by `cosineScale`. |
| `cosineScale` | false | bool | When on, forward drive power drops to 0 while the position error is over 2 × `translationalDeviationTolerance` or the heading error is over 2 × `headingDeviationTolerance`, so the robot fixes the error before it drives on. It also slows the start of a path when the robot starts off the path. |
| `minCorrectionDistance` | 0.001 | in | Dead band: position errors smaller than this get no translational correction. The controllers aren't called at all, so an integral term pauses. |

**How the power is shared out.** When the motors can't do everything at once, power is given
out in priority order: curve feedforward first, then heading feedforward plus the
`headingDriveRatio` share of heading correction, then the rest in an order set by the
tolerances:

| Situation | Order after the feedforwards |
|---|---|
| Heading over tolerance **and** position over tolerance | translational → rest of heading → drive |
| Only heading over tolerance | rest of heading → translational → drive |
| Heading within tolerance | translational → drive → rest of heading |

This only matters when the motors are saturated, which in practice means fast driving. If the
robot's heading drifts at high speed, raise `headingDriveRatio`.

## Holding the end point and finishing

| Field | Default | Units | What it does |
|---|---|---|---|
| `parametricTConstraint` | 0.025 | fraction of path | The path counts as at its end once `t ≥ 1 − this` (97.5%), and from then on the robot holds the end point. It also marks the first 2.5% of the path as its start. |
| `holdPointTranslationalScaling` | 0.45 | × | Multiplies translational correction while holding. Raise it (up to 1.0) if the robot stops an inch or two short of the end point. |
| `holdPointHeadingScaling` | 0.35 | × | Multiplies heading correction while holding. |
| `translationalConstraint` | 0.1 | in | The path is done when the distance to the end point is under this… |
| `headingConstraint` | 0.007 (≈ 0.4°) | rad | …and the heading error is under this… |
| `velocityConstraint` | 0.1 | in/s | …and the speed toward the end point is under this. |
| `timeoutConstraint` | 100 | ms | …or once this long has passed since holding started, whichever comes first. If the next path starts before the robot has settled, raise this (for example to 300-500 ms). |
