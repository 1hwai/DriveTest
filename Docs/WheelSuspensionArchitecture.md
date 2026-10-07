# DriveTest Wheel & Suspension Architecture

> This document is the implementation specification for the vehicle wheel and suspension system.
> Treat it as the source of truth for future wheel/suspension work.
>
> **Checklist convention:** completed items are marked with `[x]`. If a completed item must be changed or revisited, remove its checkmark first. Keep the checklist state updated as implementation progresses.
>
> **Work-unit convention:** implementation is tracked as `Phase N → N-1, N-2, ...`. Intermediate scope reviews are explicit work units. Do not modify implementation code before the current work unit's scope is confirmed.

## AI Workflow Requirements

For every future wheel/suspension work unit, the implementation agent must treat these project skills as mandatory companion documents:

- `.agents/skills/drivetest-engineering/SKILL.md`
- `.agents/skills/verification-evidence/SKILL.md`

**Read both before planning, editing, debugging, committing, or claiming completion.**

This document remains the source of truth for wheel/suspension architecture; the skills define how that work is reasoned about, scoped, changed, and verified.

## 1. Purpose

Replace the current wheel/suspension implementation with mechanically coherent kinematic suspension geometries, beginning with Double Wishbone and MacPherson. Each concrete geometry is implemented and validated independently before any common abstraction is introduced.

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

- [x] Determine wheel/hub kinematics.
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

## 12. Implementation Workflow

### Phase 0 — Existing System Audit

No implementation changes are made during this phase.

#### 0-1 — Current responsibility inventory

- [x] Inspect `Vehicle/Wheel.*`.
- [x] Inspect `Vehicle/Suspension.*`.
- [x] Inspect `Vehicle/Tire.*`.
- [x] Inspect `Vehicle/Car.*`.
- [x] Inspect `Vehicle/VehicleConfig.*`.
- [x] Inspect current vehicle diagnostics and physics diagnostics.
- [x] Inspect major consumers of the Wheel/Tire API.

Current findings:

- `Wheel` currently owns wheel transform, suspension raycast state, suspension force state, tire reaction torque, and wheel rotational state.
- `Suspension` currently stores scalar travel limits and spring/damper parameters, but does not represent suspension geometry.
- `Car` currently owns four independent `Wheel`, `Suspension`, and `Tire` objects and directly drives their update order.
- Front steering currently sets one steering angle on each front `Wheel`; there is no physical steering axis or upright state.
- `Tire` already has a useful separation between contact/slip state and force calculation, but currently receives its geometry through `Wheel`.
- `VehicleConfig` currently describes wheel positions as four local offsets; the replacement will need real suspension geometry.
- Debug UI, Scene diagnostics, Audio, and test code consume the existing Wheel/Tire API and therefore must be checked when that API changes.

#### 0-2 — Replacement boundary

- [x] Confirm the old `Wheel::Update()` suspension model is not the compatibility target.
- [x] Confirm control arms will be kinematic links, not independent rigid bodies.
- [x] Confirm wheel center/orientation will be outputs of suspension + steering geometry.
- [x] Confirm tire force calculation remains a separate subsystem.
- [x] Confirm the existing tire grip model may be reused where its inputs remain valid.

#### 0-3 — Initial file scope

**Core replacement files:**

```
Vehicle/Wheel.*
Vehicle/Suspension.*
Vehicle/Car.*
Vehicle/VehicleConfig.*
Vehicle/Tire.*
```

**Likely new files:**

```
Vehicle/DoubleWishbone.*
Vehicle/Upright.*
Vehicle/SteeringAssembly.*
```

Exact class/file names are not frozen until Phase 1-1.

**Consumers to update only when their required API changes:**

```
Core/Debug/DebugUI.cpp
World/Scene.cpp
Audio/AudioSystem.cpp
Tests/VehicleDiagnostics.cpp
Tests/PhysicsDiagnostics.cpp
```

**Configuration:**

```
Assets/Vehicles/TestCar/vehicle.ini
```

The configuration migration belongs to the geometry implementation phase, not the audit phase.

#### 0-4 — Implementation rule

- [x] Do not modify the implementation during Phase 0.
- [x] Before each implementation work unit, list the exact files that will be modified.
- [x] After implementation, run the relevant diagnostics before marking the work unit complete.
- [x] If a completed architecture item must be redesigned, remove its `[x]` before revising it.

### Phase 1 — Kinematic Suspension Geometry

The first concrete suspension geometry is Double Wishbone. MacPherson is implemented as a separate concrete geometry after Double Wishbone, without forcing a shared interface before the common behavior is demonstrated.

#### 1-1 — Geometry API and data model

**Decision:** use `DoubleWishbone` as the per-corner suspension assembly. Do not introduce separate `ControlArm` or `Upright` classes yet; their state is simple kinematic data owned by the assembly. A separate `SteeringAssembly` is deferred to Phase 3.

Per-corner data model:

```
DoubleWishbone
 ├─ UpperArm
 │   ├─ innerPivotA
 │   ├─ innerPivotB
 │   ├─ outerJoint
 │   ├─ innerToOuterLengthA
 │   └─ innerToOuterLengthB
 │
 ├─ LowerArm
 │   ├─ innerPivotA
 │   ├─ innerPivotB
 │   ├─ outerJoint
 │   ├─ innerToOuterLengthA
 │   └─ innerToOuterLengthB
 │
 ├─ UprightState
 │   ├─ upperJoint
 │   ├─ lowerJoint
 │   ├─ hubPosition
 │   └─ hubOrientation
 │
 └─ SpringMounts
     ├─ chassisMount
     └─ uprightMount
```

The control arms are represented by their two chassis-side pivots and fixed distances to the outer joint. The outer joints are solved points, not independent bodies.

The upright is represented by the fixed relationship between its upper/lower ball joints and hub. Its orientation is a kinematic result, not an independently simulated rigid body.

The spring/damper mounting points belong to the suspension geometry. `Suspension` continues to own scalar spring/damper parameters and force calculation; `DoubleWishbone` owns the geometry that supplies its actual length and axis.

Minimum solver math:

- Vec3 point/vector arithmetic.
- Dot/cross products.
- Vector normalization and distance.
- Rotation of local geometry by the chassis orientation.
- Circle/sphere intersection or equivalent constrained point solving for control-arm geometry.
- Construction of an upright orientation from solved joint geometry.
- No general multibody dynamics solver.

State ownership after Phase 1:

```
Car
 ├─ DoubleWishbone[4]  ← suspension geometry + solved hub state
 ├─ Suspension[4]      ← spring/damper parameters + force model
 ├─ Wheel[4]            ← wheel rotation/contact-facing state
 └─ Tire[4]             ← tire contact/slip/force model
```

`Wheel` will no longer be the owner of suspension travel geometry. Its physical wheel center/orientation will be supplied by the suspension/upright pipeline.

### Phase 1-1 implementation scope

**Files to create:**

```
Vehicle/DoubleWishbone.h
Vehicle/DoubleWishbone.cpp
```

**Files to modify:**

```
Vehicle/Car.h
Vehicle/Car.cpp
Vehicle/VehicleConfig.h
Vehicle/VehicleConfig.cpp
Assets/Vehicles/TestCar/vehicle.ini
```

Phase 1-1 itself will only introduce the data model/API and configuration representation. It will not implement the linkage solver or change vehicle dynamics.

**Files explicitly not modified in 1-1:**

```
Vehicle/Wheel.*
Vehicle/Suspension.*
Vehicle/Tire.*
Tests/*
Core/Debug/DebugUI.cpp
World/*
Audio/*
```

Those files are handled when the new API is actually integrated.

- [x] Define the per-corner suspension geometry data.
- [x] Define control-arm pivots and outer joints.
- [x] Define upright/hub state.
- [x] Define the minimum math required to solve the linkage.
- [x] Decide exact new classes/files from the Phase 0 audit.
- [x] Update this document with the confirmed Phase 1 implementation scope before coding.

#### 1-2 — Double-wishbone kinematic solver

**Implementation scope:**

Create the first real kinematic solver without changing tire forces or steering behavior. The solver owns per-corner control-arm constraints and produces the upright/hub state. Vehicle integration is limited to supplying chassis pose and consuming the solved hub state.

**Files allowed to change:**

```
Vehicle/DoubleWishbone.h
Vehicle/DoubleWishbone.cpp
Vehicle/Car.h
Vehicle/Car.cpp
Vehicle/VehicleConfig.h
Vehicle/VehicleConfig.cpp
Assets/Vehicles/TestCar/vehicle.ini
CMakeLists.txt
Tests/DoubleWishboneDiagnostics.cpp
```

**Files explicitly out of scope:**

```
Vehicle/Wheel.*
Vehicle/Suspension.*
Vehicle/Tire.*
Tests/VehicleDiagnostics.cpp
Core/Debug/DebugUI.cpp
World/*
Audio/*
```

**Verification:** build the project and run `DoubleWishboneDiagnostics`. The diagnostic must independently check arm lengths, left/right symmetry, hub position, and finite upright orientation. Compilation alone is not sufficient.

**Current status:** the solver and independent diagnostic are being revised around a coupled upper/lower upright constraint. Static geometry remains unchecked until a fresh configure/build/diagnostic run verifies the invariants.


- [x] Define the upper and lower control-arm geometry and fixed link lengths.
- [x] Define the coupled upper/lower outer-joint constraint that forms the upright.
- [x] Define the upright/hub position and orientation state.
- [x] Define left/right symmetric configuration through the existing vehicle coordinate convention.
- [x] Validate static geometry without tire forces.

#### 1-2-4 — Vehicle integration

**Scope:** connect the solved double-wishbone hub position to the existing Wheel state without changing steering, tire-force behavior, or spring/damper geometry.

**Files allowed to change:**

```
Vehicle/Car.h
Vehicle/Car.cpp
Vehicle/Wheel.h
Vehicle/Wheel.cpp
Tests/VehicleDiagnostics.cpp
Docs/WheelSuspensionArchitecture.md
```

**Integration contract:**

- [x] Car solves suspension geometry before wheel update.
- [x] Wheel receives the solved hub position from DoubleWishbone.
- [x] Wheel no longer reconstructs its physical center from the chassis-local wheel offset.
- [x] Vehicle diagnostics validate hub position to wheel position propagation.
- [ ] Wheel orientation is consumed from the solved hub state. This remains part of steering integration.

**Verification:** build and run all vehicle/physics diagnostics. The hub-to-wheel diagnostic must remain within position tolerance while the chassis changes roll/pitch.

**Current status:** implementation is committed; fresh verification is required before marking 1-2-4 complete.

#### 1-3 — Suspension travel

- [x] Implement suspension travel through the kinematic geometry.
- [x] Validate bump/rebound endpoints.
- [x] Validate continuous wheel-center motion.
- [x] Validate no artificial lateral/longitudinal wheel drift.

#### 1-4 — Double-wishbone geometry validation

- [x] Validate arm-length constraints.
- [x] Validate coupled upright constraint.
- [x] Validate left/right symmetry.
- [x] Validate finite hub position/orientation across chassis roll and pitch.
- [x] Validate travel continuity and horizontal drift limits.

**Fresh verification:** Linux build followed by ctest; PhysicsDiagnostics, VehicleDiagnostics, and DoubleWishboneDiagnostics all passed.

### Phase 1-M — Kinematic MacPherson

MacPherson is treated as its own concrete suspension geometry. Do not introduce a generic suspension interface or shared base class during this phase. Any common abstraction is deferred until both concrete geometries have been implemented and their actual shared behavior is known.

#### 1-1 — MacPherson geometry API and data model

**Decision:** use MacPherson as the per-corner suspension assembly. Do not introduce separate LowerArm, Strut, or Upright classes yet; their state is simple kinematic data owned by the assembly.

Per-corner data model:

~~~
MacPherson
 ├─ LowerArm
 │   ├─ innerPivotA
 │   ├─ innerPivotB
 │   ├─ outerJoint
 │   ├─ innerToOuterLengthA
 │   └─ innerToOuterLengthB
 │
 ├─ Strut
 │   ├─ upperMount
 │   ├─ lowerMount
 │   └─ length
 │
 └─ UprightState
     ├─ lowerJoint
     ├─ strutLowerJoint
     ├─ hubPosition
     └─ hubOrientation
~~~

The lower arm is a kinematic link constrained by its two chassis pivots and fixed link lengths to the outer joint.

The strut is the upper suspension constraint. Its upper mount is fixed to the chassis and its lower mount is attached to the upright. Strut length is a geometric constraint, not an independently simulated rigid body.

The upright/hub state is the kinematic result of the lower-arm and strut constraints. Steering-axis behavior is deferred to Phase 3.

Minimum solver math:

- Vec3 point/vector arithmetic.
- Dot/cross products.
- Vector normalization and distance.
- Rotation of local geometry by the chassis orientation.
- Constrained point solving for the lower arm.
- Strut-length constraint solving.
- Construction of an upright orientation from the solved lower joint and strut geometry.
- No general multibody dynamics solver.

**Important geometry distinction from Double Wishbone:** there is no upper control arm. The strut replaces the upper locating function and supplies the upper constraint through its chassis mount.

#### 1-1 implementation scope

**Files to modify:**

~~~
Docs/WheelSuspensionArchitecture.md
~~~

**Files explicitly not modified in 1-1:**

~~~
Vehicle/MacPherson.*
Vehicle/DoubleWishbone.*
Vehicle/Car.*
Vehicle/Wheel.*
Vehicle/Suspension.*
Vehicle/Tire.*
Vehicle/VehicleConfig.*
Tests/*
Assets/Vehicles/TestCar/vehicle.ini
~~~

1-1 is an architecture/data-model decision only. No solver, vehicle integration, or dynamics behavior changes are part of this work unit.

- [x] Define the lower-arm geometry.
- [x] Define the strut upper/lower mounts and constraint role.
- [x] Define the upright/hub state.
- [x] Define the minimum MacPherson solver math.
- [x] Decide not to create separate LowerArm/Strut/Upright classes yet.
- [x] Defer generic suspension abstraction until Double Wishbone and MacPherson have both been implemented and compared.

#### 1-2 — MacPherson kinematic solver

**Implementation scope:**

Create the first real MacPherson kinematic solver. The solver owns lower-arm and strut constraints and produces the upright/hub state. No steering or tire-force behavior changes.

**Files allowed to change:**

~~~
Vehicle/MacPherson.h
Vehicle/MacPherson.cpp
CMakeLists.txt
Tests/MacPhersonDiagnostics.cpp
~~~

**Files explicitly out of scope:**

~~~
Vehicle/Car.*
Vehicle/Wheel.*
Vehicle/Suspension.*
Vehicle/Tire.*
Vehicle/VehicleConfig.*
Assets/Vehicles/TestCar/vehicle.ini
Core/Debug/DebugUI.cpp
World/*
Audio/*
~~~

**Verification:** build the project and run MacPhersonDiagnostics. The diagnostic must independently verify lower-arm constraints, strut constraint, left/right symmetry, hub position/orientation finiteness, and static geometry.

### Phase 2 — Spring / Damper

#### 2-1 — Spring/damper integration

- [ ] Connect spring/damper to the solved suspension geometry.
- [ ] Define force direction from actual mounting points.
- [ ] Implement compression and rebound velocity.
- [ ] Implement bump/rebound mechanical limits.

#### 2-2 — Suspension validation

- [ ] Validate static ride height.
- [ ] Validate dynamic compression/rebound.
- [ ] Validate force direction.
- [ ] Validate airborne and rollover behavior.

### Phase 3 — Steering

#### 3-1 — Steering axis

- [ ] Implement upper/lower steering-axis points.
- [ ] Derive the steering axis from the suspension geometry.
- [ ] Rotate the upright/hub around the steering axis.

#### 3-2 — Steering geometry

- [ ] Add caster.
- [ ] Add KPI / SAI.
- [ ] Add scrub radius.
- [ ] Add steering arm geometry.
- [ ] Add Ackermann steering.

#### 3-3 — Steering validation

- [ ] Validate steering-axis rotation.
- [ ] Validate steering-induced hub position change.
- [ ] Validate camber/toe behavior.
- [ ] Validate extreme steering combined with suspension travel.

### Phase 4 — Tire Integration

#### 4-1 — Tire interface migration

- [ ] Connect tire contact to the new hub/wheel state.
- [ ] Preserve the existing tire grip model where appropriate.
- [ ] Remove obsolete suspension assumptions from tire inputs.

#### 4-2 — Tire force application

- [ ] Validate contact-point velocity.
- [ ] Apply tire force at the contact patch.
- [ ] Validate wheel reaction torque against the new hub state.

### Phase 5 — Vehicle Validation

#### 5-1 — Basic driving

- [ ] Validate straight-line driving.
- [ ] Validate braking.
- [ ] Validate acceleration.

#### 5-2 — Handling

- [ ] Validate cornering and load transfer.
- [ ] Validate drifting.
- [ ] Validate wheel lift.
- [ ] Validate jumps/airborne behavior.
- [ ] Validate rollover behavior.

#### 5-3 — Regression suite

- [ ] Replace obsolete suspension-axis diagnostics.
- [ ] Add double-wishbone geometry diagnostics.
- [ ] Add steering-axis diagnostics.
- [ ] Add suspension travel diagnostics.
- [ ] Add tire/hub integration diagnostics.
- [ ] Ensure all vehicle diagnostics pass.

## 13. Non-Goals

The following are not required for the first implementation:

- [ ] Physically simulated rigid-body control arms.
- [ ] Full multibody dynamics solver.
- [ ] Hydraulic steering simulation.
- [ ] Detailed bushing compliance.
- [ ] Detailed damper valve/temperature model.
- [ ] Manufacturing-level suspension modeling.

The first goal is a stable, mechanically coherent kinematic double-wishbone system that integrates cleanly with DriveTest's existing rigid-body and tire physics.
