# DriveTest Wheel & Suspension Architecture

> This document is the implementation specification for the vehicle wheel and suspension system.
> Treat it as the source of truth for future wheel/suspension work.
>
> **Checklist convention:** completed items are marked with `[x]`. If a completed item must be changed or revisited, remove its checkmark first. Keep the checklist state updated as implementation progresses.

## 1. Purpose

Replace the current wheel/suspension implementation with a mechanically coherent double-wishbone suspension model.

This is an architecture replacement, not an incremental feature addition to the existing `Wheel::Update()` implementation.

## 2. Coordinate Convention

The vehicle coordinate convention is fixed:

- Forward = +Z
- Up = +Y
- Right = -X
- Left = +X

Do not change this convention as part of the suspension rewrite.

## 3. Design Principles

- [ ] Model each wheel as part of an actual suspension assembly rather than as a point attached directly to the chassis.
- [ ] Separate suspension kinematics, steering geometry, wheel/hub state, tire contact, and tire force calculation.
- [ ] Do not preserve the current Wheel implementation merely for compatibility.
- [ ] Avoid solving the control arms as independent rigid bodies; treat them as kinematic links/constraints.
- [ ] Suspension geometry must determine wheel motion.
- [ ] Tire forces must be applied at the tire contact patch, not arbitrarily at the wheel mount.
- [ ] Ground contact must not be allowed to pull or hold the chassis down through an impossible suspension configuration.

## 4. Suspension Architecture

### 4.1 Double Wishbone

Each wheel corner consists conceptually of:

```
Chassis
 ├── Upper Control Arm ──┐
 │                       │
 └── Lower Control Arm ──┤── Upright / Knuckle ── Hub ── Wheel / Tire
                         │
                    Spring / Damper
```

The upper and lower arms constrain the upright/hub position.

- [ ] Define upper-arm chassis pivot(s).
- [ ] Define upper-arm outer joint.
- [ ] Define lower-arm chassis pivot(s).
- [ ] Define lower-arm outer joint.
- [ ] Solve upright/hub position from the arm geometry.
- [ ] Preserve left/right symmetry through configuration rather than duplicated special-case logic.

### 4.2 Suspension Axis

Suspension travel must come from the actual suspension geometry.

- [ ] Do not define suspension travel as a hard-coded world-down vector.
- [ ] Do not derive wheel travel by simply adding `direction * suspensionLength` to a chassis-mounted wheel point.
- [ ] Define the spring/damper mounting points explicitly.
- [ ] Derive the spring/damper axis from its two mounting points.
- [ ] Calculate spring/damper length from those mounting points.
- [ ] Keep suspension travel limits independent from damping force.

### 4.3 Upright / Hub

The upright connects the upper/lower control arms, steering assembly, and wheel hub.

- [ ] Define the hub position from suspension kinematics.
- [ ] Define hub orientation from the suspension and steering geometry.
- [ ] Keep wheel visual transform derived from hub state rather than independently moving the visual wheel.

### 4.4 Spring / Damper

Spring and damper behavior is separate from suspension geometry.

- [ ] Define rest length.
- [ ] Define bump travel / minimum permitted length.
- [ ] Define rebound travel / maximum permitted length.
- [ ] Define compression damping.
- [ ] Define rebound damping.
- [ ] Clamp suspension length to its mechanical limits.
- [ ] Do not use damping as a substitute for mechanical travel limits.
- [ ] Prevent a grounded wheel from pulling a rolled-over chassis toward the ground through an invalid suspension configuration.

## 5. Steering Architecture

Front-wheel steering must rotate the steering knuckle/upright around a steering axis rather than rotating the wheel in place around world/local Y.

### 5.1 Steering Axis

The steering axis is defined by the suspension's steering/ball-joint geometry.

- [ ] Define upper and lower steering-axis points.
- [ ] Derive the steering axis from those points.
- [ ] Rotate the upright/hub around the steering axis.
- [ ] Do not use a fixed world-Y axis for front-wheel steering.

### 5.2 Steering Geometry

The steering geometry should support:

- [ ] Caster.
- [ ] KPI / steering-axis inclination (SAI).
- [ ] Scrub radius.
- [ ] Steering arm geometry.
- [ ] Ackermann steering relationship.

Steering must be capable of changing hub position as a consequence of rotating around an offset/inclined steering axis.

## 6. Wheel & Tire Architecture

The conceptual update chain is:

```
Double Wishbone
      ↓
Upright / Hub
      ↓
Wheel
      ↓
Tire
      ↓
Contact Patch
      ↓
Tire Forces
```

Responsibilities:

### Suspension

- [ ] Determine wheel/hub kinematics.
- [ ] Determine suspension compression and velocity.
- [ ] Generate spring/damper forces.

### Steering

- [ ] Determine steering-axis orientation.
- [ ] Determine steering-induced hub/wheel transform.

### Wheel / Hub

- [ ] Store wheel rotational state.
- [ ] Provide wheel center/orientation to tire calculations and rendering.
- [ ] Keep visual wheel transform derived from physical hub state.

### Tire

- [ ] Determine ground contact.
- [ ] Determine contact-point velocity.
- [ ] Calculate longitudinal/lateral tire forces.
- [ ] Apply tire forces at the contact patch.
- [ ] Remain independent of the internal suspension implementation.

## 7. Suspension Kinematics

The suspension is primarily kinematic rather than a collection of independent rigid bodies.

- [ ] Solve control-arm geometry each physics update.
- [ ] Obtain wheel center from the solved upright position.
- [ ] Obtain camber from upright orientation.
- [ ] Allow suspension travel to naturally produce track-width changes.
- [ ] Allow suspension travel to naturally produce wheelbase changes where geometry causes them.
- [ ] Allow suspension geometry to determine relevant caster/camber changes rather than manually offsetting the wheel afterward.

## 8. Physics Update Order

The intended high-level update order is:

```
Chassis transform
    ↓
Suspension geometry solve
    ↓
Upright / hub transform
    ↓
Steering geometry
    ↓
Wheel center + orientation
    ↓
Ground contact
    ↓
Suspension compression
    ↓
Spring / damper force
    ↓
Tire contact velocity
    ↓
Tire forces
    ↓
Apply forces to chassis
    ↓
RigidBody integration
```

- [ ] Implement and document the final concrete update order once the new classes exist.

## 9. Vehicle Configuration

Vehicle configuration should describe physical suspension geometry rather than only wheel offsets.

### Geometry parameters

- [ ] Upper control-arm chassis pivots.
- [ ] Upper outer joint.
- [ ] Lower control-arm chassis pivots.
- [ ] Lower outer joint.
- [ ] Spring/damper mounting points.
- [ ] Steering-axis geometry.
- [ ] Hub/wheel offset.
- [ ] Steering-arm geometry.

### Suspension parameters

- [ ] Rest length.
- [ ] Bump travel.
- [ ] Rebound travel.
- [ ] Spring rate.
- [ ] Compression damping.
- [ ] Rebound damping.

### Steering parameters

- [ ] Maximum steering angle.
- [ ] Caster.
- [ ] KPI / SAI.
- [ ] Scrub radius.
- [ ] Ackermann geometry.

## 10. Edge Cases

- [ ] Full bump.
- [ ] Full rebound.
- [ ] Wheel completely airborne.
- [ ] One-wheel lift.
- [ ] Large chassis roll.
- [ ] Large chassis pitch.
- [ ] Vehicle rollover.
- [ ] Extreme steering.
- [ ] Extreme steering combined with suspension travel.
- [ ] Loss of ground contact without artificial chassis attraction.

## 11. Regression Tests

The new system should be tested independently of visual appearance.

- [ ] Static wheel geometry.
- [ ] Left/right symmetry.
- [ ] Full bump position.
- [ ] Full rebound position.
- [ ] Suspension travel continuity.
- [ ] Suspension axis correctness.
- [ ] Spring/damper force direction.
- [ ] Wheel position during chassis roll.
- [ ] Wheel position during chassis pitch.
- [ ] Steering-axis rotation.
- [ ] Steering-induced hub position change.
- [ ] Camber change through suspension travel.
- [ ] Tire contact point correctness.
- [ ] Airborne wheel behavior.
- [ ] Rollover behavior.
- [ ] No lateral/longitudinal wheel drift caused by suspension travel.

## 12. Implementation Phases

### Phase 1 — Kinematic Double Wishbone

- [ ] Implement control-arm geometry.
- [ ] Implement upright/hub position and orientation.
- [ ] Validate suspension travel without tire forces.

### Phase 2 — Spring / Damper

- [ ] Connect spring/damper to the solved suspension geometry.
- [ ] Implement bump/rebound limits.
- [ ] Validate static ride height and dynamic compression/rebound.

### Phase 3 — Steering

- [ ] Implement steering axis.
- [ ] Implement steering-axis rotation.
- [ ] Add caster/KPI/scrub-radius geometry.
- [ ] Add Ackermann steering.

### Phase 4 — Tire Integration

- [ ] Connect tire contact to the new hub/wheel state.
- [ ] Preserve the existing tire model where appropriate.
- [ ] Move tire force application to the correct contact patch.

### Phase 5 — Vehicle Validation

- [ ] Validate straight-line driving.
- [ ] Validate cornering and load transfer.
- [ ] Validate drifting.
- [ ] Validate wheel lift.
- [ ] Validate jumps/airborne behavior.
- [ ] Validate rollover behavior.

## 13. Non-Goals

The following are not required for the first implementation:

- [ ] Physically simulated rigid-body control arms.
- [ ] Full multibody dynamics solver.
- [ ] Hydraulic steering simulation.
- [ ] Detailed bushing compliance.
- [ ] Detailed damper valve/temperature model.
- [ ] Manufacturing-level suspension modeling.

The first goal is a stable, mechanically coherent kinematic double-wishbone system that integrates cleanly with DriveTest's existing rigid-body and tire physics.
