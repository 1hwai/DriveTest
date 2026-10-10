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

### 2.1 Vehicle-local reference frame and load-transfer sign convention

The coordinate convention above is a **vehicle-local** convention. It is not a world-axis convention and not a separate convention per wheel.

```text
                 +Y Up
                  ↑
                  │
      +X Left  ←  ●  →  -X Right
                  │
                  └────────→ +Z Forward
```

- `+X` is the vehicle's left side.
- `-X` is the vehicle's right side.
- `+Y` is upward.
- `+Z` is forward.
- When the chassis rotates, these axes rotate with the chassis into world space.
- `Wheel::GetWorldOrientation()` is a world-space rotation of this vehicle-local hub frame.
- The physical quaternion basis uses `+X = left`, not `-X = right`; `-X` is the derived right direction.

For cornering diagnostics the load-transfer sign is fixed as:

```text
positive steering input  → right turn  → lateral acceleration toward -X
                         → left/outside load increases
                         → leftLoad - rightLoad > 0

negative steering input  → left turn   → lateral acceleration toward +X
                         → right/outside load increases
                         → leftLoad - rightLoad < 0
```

The intended physical chain is:

```text
Tire lateral force at contact patch
            ↓
     chassis roll torque
            ↓
       chassis roll
            ↓
 left/right wheel geometry + ray distance
            ↓
 suspension compression difference
            ↓
 left/right spring normal load difference
```

A load-transfer test must therefore verify the chain rather than compensate for a missing roll response by changing spring rates, steering scale, or test thresholds.

## 3. Design Principles

- [x] Model each wheel as part of an actual suspension assembly rather than as a point attached directly to the chassis.
- [x] Separate suspension kinematics, steering geometry, wheel/hub state, tire contact, and tire force calculation.
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

### 3.1 Component Coupling

Components may be physically coupled without being implementation-dependent.

- Geometry exposes solved positions/orientations and spring mounting points.
- Suspension consumes mounting-point data and exposes spring/damper force results.
- Wheel consumes hub state rather than knowing how a concrete suspension geometry solved it.
- Tire consumes wheel/contact state rather than knowing the internal suspension implementation.
- Vehicle-level orchestration owns the order in which these results are produced and consumed.

Do not introduce additional classes solely to hide a small data transformation. The goal is stable result-oriented boundaries, not maximal decomposition.
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

- [ ] Consume ground-contact results from the replaceable contact-query boundary.
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

- [x] Implement lower-arm and strut kinematic constraints.
- [x] Produce the solved lower joint, strut lower mount, hub position, and hub orientation.
- [x] Validate static lower-arm constraints.
- [x] Validate static strut constraint.
- [x] Validate left/right symmetry and finite hub state.

**Fresh verification:** Linux configure/build followed by ctest; PhysicsDiagnostics, VehicleDiagnostics, MacPhersonDiagnostics, and DoubleWishboneDiagnostics all passed.

#### 1-3 — MacPherson suspension travel

**Implementation scope:**

Extend the MacPherson solver so suspension geometry is evaluated from chassis-local configuration in world space for the current chassis pose. Travel changes the strut length and the constrained lower-arm/strut geometry determines the resulting hub position.

**Files allowed to change:**

~~~
Vehicle/MacPherson.h
Vehicle/MacPherson.cpp
Tests/MacPhersonDiagnostics.cpp
Docs/WheelSuspensionArchitecture.md
~~~

**Files explicitly out of scope:**

~~~
Vehicle/Car.*
Vehicle/Wheel.*
Vehicle/Suspension.*
Vehicle/Tire.*
Vehicle/VehicleConfig.*
CMakeLists.txt
Core/Debug/DebugUI.cpp
World/*
Audio/*
~~~

**Verification:** build the project and run MacPhersonDiagnostics. The diagnostic must independently verify bump/rebound travel, lower-arm constraints, strut length at each travel point, left/right symmetry, travel continuity, finite hub state across chassis roll/pitch, and horizontal drift limits.

- [x] Evaluate lower-arm and strut geometry in world space from the chassis pose.
- [x] Implement bump/rebound travel through the strut constraint.
- [x] Validate lower-arm constraints throughout travel.
- [x] Validate strut constraint throughout travel.
- [x] Validate travel endpoints and continuity.
- [x] Validate left/right symmetry throughout travel.
- [x] Validate finite hub position/orientation across chassis roll and pitch.
- [x] Validate horizontal drift limits.


**Fresh verification:** Linux configure/build followed by ctest; all four diagnostics passed.\n\n#### 1-4 — MacPherson final geometry validation

**Implementation scope:**

Use the independent MacPherson diagnostic as the final validation gate for the concrete kinematic geometry before moving to spring/damper integration. This work unit does not integrate MacPherson into Car, Wheel, or Suspension.

**Files allowed to change:**

~~~
Tests/MacPhersonDiagnostics.cpp
Docs/WheelSuspensionArchitecture.md
~~~

**Files explicitly out of scope:**

~~~
Vehicle/MacPherson.h
Vehicle/MacPherson.cpp
Vehicle/Car.*
Vehicle/Wheel.*
Vehicle/Suspension.*
Vehicle/Tire.*
Vehicle/VehicleConfig.*
CMakeLists.txt
Assets/Vehicles/TestCar/vehicle.ini
Core/Debug/DebugUI.cpp
World/*
Audio/*
~~~

**Final validation requirements:**

- [x] Validate lower-arm fixed-length constraints at static pose and across travel.
- [x] Validate strut-length constraint at static pose and across travel.
- [x] Validate left/right symmetry at static pose and across travel.
- [x] Validate finite hub position and normalized hub orientation across chassis roll/pitch.
- [x] Validate bump/rebound endpoints and continuous hub motion.
- [x] Validate horizontal drift remains within the diagnostic limit.
- [x] Keep the diagnostic independent of spring/damper and tire-force behavior.

**Fresh verification:** Linux configure/build followed by ctest; PhysicsDiagnostics, VehicleDiagnostics, MacPhersonDiagnostics, and DoubleWishboneDiagnostics all passed.

### Phase 2 — Spring / Damper

Phase 2 integrates the existing scalar spring/damper model with the concrete suspension geometry without making `Suspension` responsible for suspension kinematics.

#### 2-1 — Spring/damper responsibility and data flow

**Goal:** define the exact ownership and data flow before modifying `Suspension` or reconnecting it to vehicle physics.

The separation is:

```
Suspension Geometry
        ↓
Spring/Damper mount A + mount B
        ↓
Actual spring length + spring axis
        ↓
Suspension compression
        ↓
Compression velocity
        ↓
Spring + damper force
        ↓
Force along spring axis
        ↓
Chassis / suspension attachment
```

**Responsibilities:**

`DoubleWishbone` / `MacPherson`
- Own spring/damper mounting geometry.
- Solve the current mount positions from chassis pose and suspension travel/geometry.
- Provide the actual spring length and axis.
- Do not calculate spring rate, damping, or force magnitude.

`Suspension`
- Own rest length, bump/rebound travel, spring rate, and damping rates.
- Convert actual spring length into compression.
- Convert compression change into compression/rebound velocity.
- Apply mechanical length limits.
- Calculate spring/damper force magnitude.
- Do not own wheel position, ground contact, suspension linkage geometry, or steering geometry.
- Do not derive suspension length from a ground raycast.

The spring/damper force direction is derived from the two actual mounting points. It is not a fixed chassis-local or world-down vector.

**State/data contract:**

```
Geometry:
    mountA
    mountB
        ↓
    length = distance(mountA, mountB)
    axis   = normalize(mountB - mountA)

Suspension:
    previousLength
    currentLength
        ↓
    compression = restLength - currentLength
    compressionVelocity = (previousCompression - currentCompression) / dt
        ↓
    forceMagnitude = spring + damper
        ↓
    force = axis * forceMagnitude
```

The sign convention must be defined so positive compression velocity means increasing compression and selects compression damping; decreasing compression selects rebound damping. The implementation must preserve this convention explicitly rather than relying on ambiguous variable names.

**Mechanical limits:**

- Minimum spring length = `restLength - bumpTravel`.
- Maximum spring length = `restLength + reboundTravel`.
- Actual spring length is clamped to those mechanical limits before compression is calculated.
- Damping must not be used to enforce travel limits.
- A wheel with no valid ground contact must not acquire a ground-derived spring compression merely because the chassis is rotated or overturned.

**Important integration boundary:**

Phase 2-1 does not decide how the spring force is applied to the rigid body in `Car`, nor does it migrate tire/contact behavior. That belongs to the subsequent integration work.

**Files allowed to change:**

```
Docs/WheelSuspensionArchitecture.md
```

**Files explicitly out of scope:**

```
Vehicle/Suspension.h
Vehicle/Suspension.cpp
Vehicle/DoubleWishbone.*
Vehicle/MacPherson.*
Vehicle/Car.*
Vehicle/Wheel.*
Vehicle/Tire.*
Vehicle/VehicleConfig.*
Tests/*
Assets/Vehicles/TestCar/vehicle.ini
Core/Debug/DebugUI.cpp
World/*
Audio/*
CMakeLists.txt
```

**Verification for 2-1:** architecture review only. No implementation or runtime verification is claimed. The next work unit must implement the smallest API/data changes required by this contract and then verify the spring/damper invariants with an independent diagnostic.

- [x] Confirm spring/damper ownership and data flow.
- [x] Confirm actual mounting-point-derived length and axis.
- [x] Confirm compression/rebound velocity sign convention.
- [x] Confirm mechanical travel limits are separate from damping.
- [x] Confirm ground contact is not the source of suspension length.

#### 2-2 — Spring/damper model implementation

- [x] Implement the Phase 2-1 responsibility/data contract in `Suspension`.
- [x] Track actual spring length/compression state across physics updates.
- [x] Calculate force magnitude from spring compression and compression/rebound damping.
- [x] Preserve mechanical travel limits independently from damping.

#### 2-3 — Geometry ↔ Suspension force connection

**Scope:** connect the concrete suspension geometries to the scalar `Suspension` model through actual spring/damper mounting points. Rigid-body force application remains deferred to vehicle integration.

- [x] Connect Double Wishbone and MacPherson spring mounts to `Suspension`.
- [x] Derive force direction from actual spring mounts.
- [x] Expose the resulting force for later application at the geometry-defined attachment points.
- [ ] Remove obsolete ground-raycast suspension-force assumptions; this remains deferred until the new suspension pipeline is integrated into `Car`/`Wheel`.

#### 2-4 — Suspension validation

**Scope:** independently validate the spring/damper model and geometry-to-suspension mount data before rigid-body force application is integrated. This work unit does not modify `Car`, `Wheel`, `Tire`, or rigid-body integration.

**Files allowed to change:**

~~~
Tests/SuspensionDiagnostics.cpp
Tests/DoubleWishboneDiagnostics.cpp
Tests/MacPhersonDiagnostics.cpp
Docs/WheelSuspensionArchitecture.md
~~~

**Files explicitly out of scope:**

~~~
Vehicle/Suspension.*
Vehicle/DoubleWishbone.*
Vehicle/MacPherson.*
Vehicle/Car.*
Vehicle/Wheel.*
Vehicle/Tire.*
Vehicle/VehicleConfig.*
World/*
Audio/*
Core/Debug/*
~~~

**Validation requirements:**

- [x] Validate mechanical bump/rebound limits.
- [x] Validate compression and compression velocity.
- [x] Validate compression/rebound damping selection.
- [x] Validate non-tensile spring/damper force behavior.
- [x] Validate actual mount-derived spring length.
- [x] Validate force-vector direction from the actual mount delta.
- [x] Validate degenerate zero-length mounts without NaN/Inf.
- [x] Validate Double Wishbone spring mount finiteness, symmetry, and non-degenerate length.
- [x] Validate MacPherson spring mount finiteness, symmetry, and non-degenerate length.
- [x] Keep diagnostics independent of rigid-body force application and tire behavior.

**Force-direction convention:** `CalculateForceVector(mountA, mountB)` returns `normalize(mountB - mountA) * magnitude`. The vector therefore points from mount A toward mount B. Which body receives this vector is intentionally deferred to vehicle force integration; the diagnostic verifies the geometric direction only.

**Fresh verification:** Linux clean build followed by ctest; all registered diagnostics must pass before this work unit is considered complete.
### Phase 3 — Steering

#### 3-1 — Steering axis

- [x] Define the steering axis from the concrete suspension geometry.
- [x] Derive the Double Wishbone axis from the solved upper/lower outer joints.
- [x] Derive the MacPherson axis from the solved upper strut mount/lower outer joint.
- [x] Rotate upright/hub state around the current world-space steering axis.
- [x] Preserve zero-steering suspension state.
- [x] Validate finite axis and axis-preserving hub rotation.

The steering axis is a result of the current suspension pose, not a fixed world-Y axis and not a standalone class. Caster and KPI/SAI emerge from the placement of the axis endpoints rather than being separate 3-1 inputs.

#### 3-2 — Steering geometry

- [x] Caster-related steering-axis placement.
- [x] KPI / SAI-related steering-axis placement.
- [x] Scrub-radius-related hub offset.
- [x] Steering-arm geometry relationship.
- [x] Hub transform changes caused by steering-axis rotation.

These geometry properties are represented through the existing concrete suspension geometry and hub state. No separate steering-axis class was introduced.

#### 3-3 — Steering state ↔ suspension geometry integration

- [x] Connect Wheel steering state to front suspension geometry.
- [x] Correct the left/right mirror convention.
- [x] Propagate steering-induced hub position and orientation.
- [x] Validate steering geometry with independent diagnostics.
- [x] Validate steering direction together with cornering load-transfer behavior.

#### 3-4 — Ackermann steering

- [x] Calculate inner/outer steering angles from wheelbase and track width.
- [x] Apply the larger steering magnitude to the inner wheel for both turn directions.
- [x] Keep both front geometry angles aligned with the actual solved steering-axis turn direction.
- [x] Validate right-turn and left-turn Ackermann symmetry.
- [x] Validate finite steering-angle results.
- [x] Validate actual Ackermann steering direction in regression diagnostics.

**Current status:** Phase 3 steering-axis, steering-geometry, integration, and Ackermann work units are complete and covered by vehicle diagnostics.
### Phase 4 — Tire Integration

#### 4-1 — Tire interface migration

- [x] Connect tire contact to the new hub/wheel state.
- [x] Preserve the existing tire grip and steering-force basis.
- [x] Keep tire force behavior unchanged while the new hub state is introduced.
- [x] Store hub position and hub orientation together in Wheel state.
- [x] Drive wheel rendering from the solved hub orientation instead of independent steering rotation.
- [x] Verify hub state propagation with diagnostics.

**Implementation files:**

```
Vehicle/Wheel.h
Vehicle/Wheel.cpp
Vehicle/Tire.cpp
Vehicle/Car.cpp
World/Scene.cpp
Tests/VehicleDiagnostics.cpp
```

**Current status:** interface/state migration is complete and regression-safe. Tire force-basis migration to solved hub orientation is intentionally deferred until the dedicated tire-force work unit.

#### 4-2 — Tire contact patch / force basis migration

**Goal:** make the tire contact basis derive from the solved wheel hub orientation while preserving the existing tire force model itself.

The responsibility boundary is:

```
Suspension Geometry
        ↓
   Hub Position
   Hub Orientation
        ↓
       Wheel
        ↓
 Contact Point + Normal
        ↓
 Project hub forward onto contact plane
        ↓
 Tire contact basis
   ├─ forward
   └─ lateral
        ↓
 Existing slip / grip / force model
```

**Important coordinate detail:** DriveTest uses vehicle-right = `-X`, while the quaternion rotation basis is right-handed. Therefore the hub orientation must be constructed from the physical `+X = left` axis, `+Y = up`, and `+Z = forward`. Using vehicle-right (`-X`) directly as a quaternion basis axis creates a reflection rather than a rotation.

**Tire state basis:**

- `forward = hubOrientation * VehicleCoordinates::Forward()`.
- Project `forward` onto the contact plane using the contact normal.
- Normalize the projected forward vector.
- `lateral = forward × contactNormal`, producing vehicle-right (`-X`) for a flat road.
- Do not apply `Wheel::GetSteeringAngle()` again after reading the solved hub orientation; steering is already represented by the hub orientation.

**Force-model boundary:**

- Keep the existing slip-ratio calculation.
- Keep the existing slip-angle calculation.
- Keep the existing grip curve.
- Keep the existing friction limit and combined-slip scaling.
- Keep rolling resistance behavior.
- Only migrate the coordinate basis used to interpret contact velocity and apply the resulting force.

**Verification:**

- [x] Correct the hub orientation basis so it represents a proper quaternion rotation under the vehicle coordinate convention.
- [x] Derive tire contact forward/lateral basis from solved hub orientation.
- [x] Prevent steering from being applied twice.
- [x] Add an independent diagnostic comparing the tire basis with the projected solved hub basis.
- [x] Run the full Linux build and vehicle diagnostics after the migration.

**Current status:** solved-hub tire contact basis migration is complete. The user verified the Linux build and both registered ctest diagnostics pass (PhysicsDiagnostics and VehicleDiagnostics). This confirms the current regression suite, not yet slope/edge/jump-landing contact-query behavior; that is explicitly addressed by Phase 4-3.

### Phase 4-3 — Replaceable Wheel Contact Architecture

**Goal:** make wheel-ground contact discovery a replaceable implementation, not a permanent design choice. Raycast is only one candidate and must not define the interface.

This work unit is an architecture specification. It does not select the final contact algorithm and does not claim the runtime has already been refactored.

#### Non-negotiable design objective

A developer must be able to add a newly discovered contact algorithm and select it without editing `Wheel`, `Suspension`, `Tire`, or vehicle-force consumers. Algorithm-specific code belongs behind one stable provider boundary. Consumers depend only on the result contract.

Candidate providers include, without preference or commitment:

- Single ray / adjusted ray.
- Multi-ray or other discrete samples.
- Sphere cast, shape cast, or swept volume.
- Tire-shaped or multi-point geometric contact.
- A node/beam-based or other future tire-contact implementation, if it can be adapted to the agreed contract.
- Any future algorithm not listed here.

The list is deliberately open-ended. Do not add algorithm-specific branches such as `if (isRaycast)` to consumers.

#### Target dependency structure

```text
Vehicle / Car composition root
    └── selects and injects one IWheelContactProvider
                  |
                  v
Wheel geometry + generic query input
                  |
                  v
       IWheelContactProvider
       ├── Raycast provider
       ├── Multi-sample provider
       ├── Shape/sweep provider
       └── future/custom provider
                  |
                  v
        WheelContactResult
                  |
          ┌───────┴────────┐
          v                v
     Suspension           Tire
   uses suspension     uses contact
   geometry/state      points/normals/
   for spring length   surface/velocity
                          /
                         /
            Vehicle force orchestration
```

The diagram describes the intended dependency direction, not current implementation. Concrete provider selection belongs in vehicle/application composition or configuration. `Wheel`, `Suspension`, and `Tire` must not construct, select, or identify a concrete provider.

#### Interface and data contract

The implementation should introduce a provider-neutral interface (names are provisional until the code API review), conceptually:

```cpp
class IWheelContactProvider {
public:
    virtual ~IWheelContactProvider() = default;
    virtual WheelContactResult Query(
        const WheelContactInput& input,
        const PhysicsWorld& physicsWorld,
        const RigidBody* ignoredBody
    ) const = 0;
};
```

The interface above now matches the first implementation. The provider receives a read-only PhysicsWorld query context and collision-filter target; Wheel only forwards that context and never invokes a concrete query. A separate world adapter is intentionally deferred until an alternative provider demonstrates a concrete missing capability. If that happens, add the capability at this boundary instead of adding algorithm-specific branches to vehicle consumers.

The contract must meet these requirements:

- **Generic input:** wheel/hub pose and orientation, relevant tire dimensions/shape description, collision filtering, and algorithm-neutral query limits/settings. Do not make a ray, ray direction, or ray hit distance the universal input model.
- **Stable result:** explicit contact state; zero or more contact samples; each sample's world-space point and corresponding normal; available surface/material identity; and optional separation/penetration or feature metadata when the underlying query can provide it.
- **Explicit capabilities:** where a strategy cannot supply a field, represent that limitation explicitly rather than inventing values or silently changing semantics. Consumers must not branch on the concrete algorithm type.
- **Consistent semantics:** point, normal, surface identity, and optional distance/penetration data must describe the same contact sample. Define coordinate space, normal direction, units, validity, ordering, and empty-result behavior.
- **Multiple contacts:** the result type must be able to represent multiple samples even if the first provider returns one. Do not force all strategies to collapse to one arbitrary point at the interface boundary.
- **Ownership/lifetime:** result data must remain valid independently of temporary query buffers or provider internals.
- **No hidden force model:** the provider discovers/describes contact. Tire-force calculation and chassis-force application remain separately owned responsibilities unless a future architecture review explicitly changes that boundary.

Do not expose implementation-specific types such as `RaycastResult`, ray parameters, cast handles, or provider-private buffers through the public wheel/tire contact contract. The current Raycast provider fills one sample; Wheel stores the complete result but the legacy tire-force path still consumes the first sample only. Multi-point force aggregation is not implemented yet. The current suspension length also remains a transitional hub-to-contact geometric estimate, not a true spring-mount length; that decoupling is a separate unfinished step.

#### Responsibility boundaries

- **Contact provider:** performs contact discovery and provider-specific candidate filtering/selection. It owns ray/shape/sample details and any algorithm-specific tolerances.
- **Contact-query world access:** the current provider receives `PhysicsWorld` as a read-only query context. `Wheel` forwards it without calling `Raycast` or inspecting collision shapes. If an alternative provider needs a missing world capability, expose it at this boundary (through a neutral adapter or physics-world API) rather than leaking it into `Wheel`, `Suspension`, or `Tire`.
- **Wheel / hub:** owns wheel rotational state and exposes the contact result through a stable API. It does not implement ground-search logic.
- **Suspension geometry:** solves hub position/orientation and actual spring/damper mount positions.
- **Suspension model:** computes spring length from the actual spring/damper mount points, and computes compression, velocity, and spring/damper forces from its own geometry/state. It must not infer spring length from contact distance.
- **Tire:** consumes contact state/samples, wheel/hub state, contact-point velocity, and surface data to calculate tire state and forces. It does not query the ground itself and does not know which provider produced the result.
- **Car / vehicle orchestration:** injects the chosen provider, sequences updates, and applies suspension and tire forces at the appropriate mechanical application points. It does not contain per-algorithm logic.
- **Diagnostics:** tests provider contracts and vehicle behavior without requiring the consumer code to know which provider is active.

#### The replaceability test

The architecture is not complete merely because an interface exists. It must pass this practical test:

1. Add a new provider in its own implementation files.
2. Register/select it through the composition/configuration boundary.
3. Run the same consumer-level tests with the old and new providers.
4. Confirm that `Wheel`, `Suspension`, `Tire`, their public consumer APIs, and force-model code did not need algorithm-specific edits.

If adding a provider requires changes to `Wheel`, `Suspension`, or `Tire`, the boundary has leaked implementation details and must be redesigned before adding more algorithms.

There is one honest limit: a provider can be a drop-in implementation only when the engine exposes the world data and operations it needs. If a future method requires a new collision-world capability, add that capability behind the world adapter; do not push the new algorithm into wheel/suspension/tire consumers. A fundamentally different deformable node/beam tire model may also require a separate model-level design, because it changes more than how a contact point is discovered. That possibility must not be used as a reason to couple ordinary contact providers to the consumers.

#### Suspension and tire invariants

- A world-down ray is not the definition of wheel-ground contact.
- Contact discovery does not determine suspension kinematics.
- Spring/damper length comes from the real geometry's mount points, never from ray hit distance or a provider-specific distance.
- A wheel may have geometric contact while its normal load is zero; preserve valid contact geometry/basis without inventing tire force.
- No valid contact means no terrain-derived tire force.
- Contact samples must preserve consistent point/normal pairing and must apply resulting forces at the intended contact locations. If samples are aggregated, document how the resulting force and torque are preserved.
- Keep the vehicle coordinate convention: +X left, -X right, +Y up, +Z forward.
- Do not mask contact errors by changing spring rate, steering sign/scale, friction, or test thresholds.
- Changing providers must not silently change units, coordinate spaces, contact-state meaning, or normal orientation.

#### Strategy selection is intentionally deferred

Do not select an algorithm by name, reputation, or assumption. After the boundary and existing world-query capabilities are audited, compare candidates against the same tests. Consider correctness on slopes, banks, crests, dips, triangle boundaries, edges, wheel lift, and jump landing, alongside stability, determinism, performance, and implementation cost.

Raycast may remain as the first adapter to preserve a baseline and isolate the refactor. That does **not** make Raycast the preferred long-term strategy or allow its semantics to leak into the contract.

#### Implementation sequence

1. **Audit existing APIs:** inspect terrain, road, shape-collision, `PhysicsWorld`, and current `Wheel::Update()` query/force paths.
2. **Specify the contract:** define input, result, contact-state semantics, multiple samples, optional capabilities, coordinate spaces, units, and empty-result behavior.
3. **Introduce the boundary:** add the provider interface and a provider-neutral contact result without changing tire grip behavior. Defer a separate world adapter until a real alternative provider requires a capability the current PhysicsWorld API cannot supply.
4. **Wrap the current query:** implement the existing behavior as one provider behind the interface; do not improve its algorithm in the same work unit unless required for correctness.
5. **Inject the provider:** select it outside `Wheel`, `Suspension`, and `Tire`; remove concrete-query construction and algorithm checks from consumers.
6. **Decouple suspension:** connect actual suspension geometry/mount-point calculations to the runtime path and remove legacy ray-distance-to-spring-length coupling.
7. **Add contract tests:** use a deterministic fake provider to test no contact, one contact, multiple contacts, changing normals, and missing optional data.
8. **Add scene diagnostics:** flat ground and left/right symmetry; slopes and banks; crest/dip; terrain/road edges and triangle transitions; front-wheel-first slope landing; one or more airborne wheels; contact point/normal pairing; force application point and resulting torque.
9. **Compare providers:** only after the same test harness works, implement and compare alternative candidates. Keep each provider in separate files and selectable without consumer edits.

Each implementation work unit must list exact files before editing, update this checklist only after evidence, and follow the mandatory engineering/verification skills above.

#### Work checklist

- [x] Record the goal that any future contact algorithm must be replaceable behind a provider boundary.
- [x] Record consumer independence as a hard architecture requirement.
- [x] Keep strategy selection open, including algorithms discovered in the future.
- [x] Define the intended provider-neutral contract requirements and replaceability test.
- [x] Audit the existing query API and the actual Wheel/Car runtime call path.
- [x] Finalize and implement the provider-neutral input/result contract.
- [x] Add the provider interface and default Raycast provider; defer a separate world adapter until an alternative needs it.
- [x] Move the existing contact query behind the provider boundary.
- [x] Allow Car to receive an injected provider without changing Wheel/Suspension/Tire consumer code.
- [ ] Remove the transitional contact-geometry-to-spring-length coupling and use real suspension mount-point kinematics.
- [x] Add the baseline provider-contract diagnostic.
- [ ] Add slope/edge/landing diagnostics that validate contact behavior independently of the active provider.
- [ ] Implement and compare alternative providers without editing consumer algorithms.

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

### MacPherson Kinematics Diagnostic — MP-KIN-09

- [x] Add one bounded search for the upper feasible-travel boundary using the existing synthetic diagnostic configuration.
- [x] Check that the final lower bracket endpoint is accepted and the upper endpoint is rejected.
- [x] Check that rejection at the upper endpoint preserves the last valid geometry.
- [x] Local execution passed: `travelBoundaryBracket=1`; valid endpoint `0.243311`, invalid endpoint `0.243311` (difference below displayed precision), bracket width `1.49012e-08`; failure preserved the last valid geometry. The complete MacPherson diagnostic reported `[PASS]`.


The search brackets the transition between a known accepted travel of `+0.075` and a known rejected travel of `+0.30`, using 24 bisection iterations. The bracket width must be below `0.000001`. This characterizes only this synthetic configuration and the current solver's acceptance boundary; it is not a real vehicle travel limit or proof of all singular configurations. Once this diagnostic passes, stop adding routine MacPherson kinematics tests and proceed to configuration validation / runtime integration as planned.

## 13. Non-Goals

The following are not required for the first implementation:

- [ ] Physically simulated rigid-body control arms.
- [ ] Full multibody dynamics solver.
- [ ] Hydraulic steering simulation.
- [ ] Detailed bushing compliance.
- [ ] Detailed damper valve/temperature model.
- [ ] Manufacturing-level suspension modeling.

The first goal is a stable, mechanically coherent kinematic double-wishbone system that integrates cleanly with DriveTest's existing rigid-body and tire physics.
