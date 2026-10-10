# Wheel & Suspension Recovery — Findings and Solution Log

> Companion to `WheelSuspensionRecoveryPlan.md`. This is a living evidence log, not a second architecture specification.
>
> Record findings and agreed remedies phase by phase. Do not fill unknown fields with guesses. Keep the Korean log synchronized.

## Recording rules

- Cite exact paths, symbols, line ranges where available, test names, commands, commit/revision, and relevant output.
- Separate **observed fact**, **interpretation**, **proposed remedy**, and **agreed decision**.
- A proposed remedy is not approved merely because it appears here; record agreement before implementation.
- Record implementation status separately from verification status.
- When evidence changes, append a dated/revisioned update rather than silently deleting the prior conclusion.
- Do not mark a remedy complete until fresh verification evidence is recorded.

## Phase 5 — Implementation audit

**Status:** Not started  
**Audited revision:** TBD  
**Audit date:** TBD

### 5.1 Requirements traceability

- **Observed facts:** Not audited.
- **Evidence:** Pending.
- **Assessment:** Pending.
- **Proposed remedy:** Pending audit.
- **Agreed decision:** Pending review.
- **Affected files:** Pending.
- **Verification:** Pending.

### 5.2 Kinematic/mechanical correctness

- **Observed facts:** Not audited.
- **Evidence:** Pending.
- **Assessment:** Pending.
- **Proposed remedy:** Pending audit.
- **Agreed decision:** Pending review.
- **Affected files:** Pending.
- **Verification:** Pending.

### 5.3 Spring/damper and force path

- **Observed facts:** Not audited.
- **Evidence:** Pending.
- **Assessment:** Pending.
- **Proposed remedy:** Pending audit.
- **Agreed decision:** Pending review.
- **Affected files:** Pending.
- **Verification:** Pending.

### 5.4 Responsibility and data ownership

- **Observed facts:** Not audited.
- **Evidence:** Pending.
- **Assessment:** Pending.
- **Proposed remedy:** Pending audit.
- **Agreed decision:** Pending review.
- **Affected files:** Pending.
- **Verification:** Pending.

### 5.5 Ground-contact boundary

- **Observed facts:** Not audited.
- **Evidence:** Pending.
- **Assessment:** Pending.
- **Proposed remedy:** Pending audit.
- **Agreed decision:** Pending review.
- **Affected files:** Pending.
- **Verification:** Pending.

### 5.6 Vehicle runtime integration

- **Observed facts:** Not audited.
- **Evidence:** Pending.
- **Assessment:** Pending.
- **Proposed remedy:** Pending audit.
- **Agreed decision:** Pending review.
- **Affected files:** Pending.
- **Verification:** Pending.

### 5.7 Verification inventory

- **Observed facts:** Not audited.
- **Evidence:** Pending.
- **Assessment:** Pending.
- **Proposed remedy:** Pending audit.
- **Agreed decision:** Pending review.
- **Affected files:** Pending.
- **Verification:** Pending.

### Phase 5 exit review

- Requirements traced: Pending.
- Main defects and evidence recorded: Pending.
- Proposed correction order: Pending.
- Implementation untouched during audit: Pending confirmation.
- User review / agreement: Pending.

## Phase 6 — Correct kinematic models

**Status:** Blocked until Phase 5 findings and scope are agreed.

For each work unit, add an entry with:
- Work-unit ID and goal
- Evidence-backed defect
- Agreed correction
- Exact changed files
- Geometry invariants and test scenarios
- Fresh command/output/revision
- Remaining limitations

## Phase 7 — Responsibility and interface boundaries

**Status:** Blocked until audit findings and dependencies are agreed.

Record the before/after ownership matrix, data contract, changed files, and boundary tests.

## Phase 8 — Replaceable ground-contact query

**Status:** Blocked until contact-boundary findings and scope are agreed.

Record query/result semantics, algorithm substitution or composition tests, edge-case results, and limitations.

## Phase 9 — Vehicle runtime integration

**Status:** Blocked until geometry and component contracts are sufficiently validated.

Record the actual update order, force application points/directions, runtime symptoms, diagnostic evidence, and regression results.

## Phase 10 — Validation and regression

**Status:** Blocked until the integrated work units are ready for system-level validation.

Record the tested revision, environment, exact commands/scenarios, relevant outputs, failures, known approximations, and exit decision.

## Reusable work-unit entry

### [Work-unit ID] — [Short title]

- **Date / revision:**
- **Problem and observed facts:**
- **Evidence:**
- **Interpretation and confidence:**
- **Proposed remedy:**
- **Agreed decision:**
- **Files in scope:**
- **Files changed:**
- **Expected result / invariant:**
- **Verification command(s):**
- **Observed verification result:**
- **Status:** Not started / In progress / Verified / Failed / Unverified
- **Remaining limitations / next dependency:**


## Phase 5 — Initial source audit update

**Status:** In progress — source review started; exit criteria not met.  
**Audited revision:** b247b5f2affd639cbaa69bea45f6eee1bb9e251a (suspension-runtime-integration).  
**Audit date:** 2026-10-09.  
**Implementation changes during audit:** None.

### 5.1 Requirements traceability — preliminary

- **Observed facts:**
  - The architecture document contains extensive unchecked design requirements, while several implementation subphases are marked complete.
  - Double Wishbone and MacPherson have separate geometry classes and separate diagnostics source files.
  - The current vehicle runtime is typed directly to DoubleWishbone; Car.h exposes GetSuspensionGeometry() returning DoubleWishbone&.
  - CMakeLists.txt registers PhysicsDiagnostics and VehicleDiagnostics in the existing CI workflow. MacPherson, Double Wishbone, and Suspension diagnostics have separate opt-in build options and are not enabled in .github/workflows/physics-diagnostics.yml.
- **Evidence:** Docs/WheelSuspensionArchitecture.md; Vehicle/Car.h; Vehicle/Car.cpp; CMakeLists.txt; .github/workflows/physics-diagnostics.yml; Tests/DoubleWishboneDiagnostics.cpp; Tests/MacPhersonDiagnostics.cpp; Tests/SuspensionDiagnostics.cpp.
- **Assessment:** Partially implemented / verification not established for the complete historical scope. Existing checkmarks and prose claiming prior successful runs are historical claims, not fresh evidence for the audited revision.
- **Proposed remedy:** Build a requirement-by-requirement traceability matrix before altering old checkboxes. Add focused diagnostics to the repeatable verification path after reviewing their validity.
- **Agreed decision:** Pending user review.
- **Affected files:** No implementation files changed. Audit log only.
- **Verification:** Repository source and the current revision's recorded GitHub Actions run inspected.

### 5.2 Kinematic/mechanical correctness — preliminary

- **Observed facts:**
  - DoubleWishbone::SolveConstraints() preserves each arm's two pivot-to-outer-joint distances and the upper/lower upright-joint distance, using a sampled search over upper-joint Y and a bracket refinement.
  - DoubleWishbone::SolveAtTravel() parameterizes travel by changing the target average local Y of the two outer joints. This is a constrained kinematic parameterization, but it is not by itself proof that contact constraints and all desired suspension geometry are satisfied.
  - Car::UpdatePhysics() chooses travel by comparing only the chassis-local Y coordinate of the candidate hub with the target hub position derived from contact point + normal * wheel radius. It does not minimize or validate the full 3D hub-to-target error.
  - MacPherson has a standalone solver and diagnostic source, but no current Car runtime path selects or owns a MacPherson instance.
- **Evidence:** Vehicle/DoubleWishbone.cpp (SolveAtTravel, SolveConstraints, SolveArmAtHeight); Vehicle/Car.cpp (contact target and travel search); Vehicle/MacPherson.cpp; Vehicle/Car.h.
- **Assessment:** Double Wishbone solver exists, but full mechanical/contact correctness remains unverified. MacPherson geometry code exists, but runtime integration is not implemented in the current Car path.
- **Proposed remedy:** Add independent checks for full 3D contact residual, linkage constraints across travel and chassis poses, and explicit runtime-selection coverage for each concrete geometry. Do not select a final correction until the residuals and test behavior are measured.
- **Agreed decision:** Pending user review.
- **Affected files:** To be determined after audit.
- **Verification:** Current CI does not run the standalone geometry diagnostics on this revision.

### 5.3 Spring/damper and force path — preliminary

- **Observed facts:**
  - Suspension::ClampLength() enforces configured scalar bounds, but Suspension::UpdateFromMounts() computes the actual mount distance and compression without calling ClampLength().
  - Runtime Wheel::Update() uses UpdateFromMounts(), then calculates the spring/damper force vector from the two geometry mount points and applies the negated vector to the chassis at mount A.
  - Runtime spring/damper integration receives mount points from Double Wishbone. MacPherson mount outputs are not consumed by Car.
  - The wheel's reported normal load is the spring force magnitude projected against the contact normal; this is not a separate solution of a full wheel/upright force equilibrium.
  - The vehicle diagnostic captures a suspension energy residual, but the existence of that metric alone does not establish energy conservation.
- **Evidence:** Vehicle/Suspension.cpp (ClampLength, UpdateFromMounts, CalculateForceVector, CalculateForce); Vehicle/Wheel.cpp (Wheel::Update); Vehicle/Car.cpp; Tests/VehicleDiagnostics.cpp.
- **Assessment:** Mount-derived length and force direction are implemented in the active Double Wishbone path. Mechanical limit enforcement through the active mount-update path is not demonstrated and conflicts with the documented 2-1/2-2 contract. Complete force balance remains unverified.
- **Proposed remedy:** Decide whether travel limits constrain geometric travel, spring length, or both, and encode that contract explicitly. Add tests that exercise the exact runtime method and independently check force direction, compression velocity sign, and energy/work consistency.
- **Agreed decision:** Pending user review.
- **Affected files:** To be determined after audit.
- **Verification:** The current CI run's PhysicsDiagnostics energy check fails; see 5.7.

### 5.4 Responsibility and data ownership — preliminary

- **Observed facts:**
  - Car owns and orders geometry, wheel, suspension, and tire updates.
  - Wheel::Update() takes both Suspension& and const DoubleWishbone&, queries ground contact, updates suspension mount state, calculates spring/damper values, and applies the suspension force to the chassis.
  - This makes Wheel aware of a concrete geometry type and responsible for work beyond wheel rotational state/contact-facing data.
  - The debug telemetry in Car.cpp computes a legacy expected wheel displacement from a fixed (0,-1,0) axis and suspension length even though the active hub position comes from Double Wishbone geometry.
- **Evidence:** Vehicle/Car.h; Vehicle/Car.cpp; Vehicle/Wheel.h; Vehicle/Wheel.cpp; Vehicle/Suspension.h; Vehicle/Suspension.cpp.
- **Assessment:** Incorrectly connected relative to the architecture's stated responsibility boundaries. The stale fixed-axis telemetry is not a valid geometry residual for the current model.
- **Proposed remedy:** After the audit, make Car the orchestrator, keep geometry solving in the concrete geometry, keep scalar spring/damper state/force calculation in Suspension, and narrow Wheel to wheel state plus consuming contact results. Replace legacy telemetry with geometry-aware residuals.
- **Agreed decision:** Pending user review.
- **Affected files:** To be determined after audit.
- **Verification:** Source-level finding; no implementation changes made.

### 5.5 Ground-contact boundary — preliminary

- **Observed facts:**
  - IWheelContactProvider is an injectable interface owned by the Car composition path; the default provider is raycast-based. This is a real replacement boundary at the provider selection level.
  - RaycastWheelContactProvider::Query() casts in fixed world-down direction (0,-1,0).
  - Both Car and Wheel consume only contact.samples.front(). Returning multiple samples from a provider therefore does not currently make the consumers perform multi-point contact handling.
  - WheelContactSample includes optional surface/separation/penetration fields, but the default raycast provider only fills point, normal, and query distance.
  - The provider contract diagnostic checks a single plane hit; it does not validate sloped surfaces, edge transitions, multiple contacts, or rollover behavior.
- **Evidence:** Vehicle/IWheelContactProvider.h; Vehicle/WheelContact.h; Vehicle/RaycastWheelContactProvider.cpp; Vehicle/Car.cpp; Vehicle/Wheel.cpp; Tests/VehicleDiagnostics.cpp.
- **Assessment:** Provider injection is implemented; multi-sample consumption and difficult-contact behavior are unverified/not implemented in current consumers.
- **Proposed remedy:** Keep the provider boundary, explicitly define single- versus multi-sample semantics, and test sloped/edge/airborne/rollover cases. Do not commit to a specific replacement algorithm before these tests establish what the current query gets wrong.
- **Agreed decision:** Pending user review.
- **Affected files:** To be determined after audit.
- **Verification:** Current provider contract diagnostic passed in CI, but it only demonstrates the basic single-plane contract.

### 5.6 Vehicle runtime integration — preliminary

- **Observed facts:**
  - Runtime update order is implemented directly in Car::UpdatePhysics(): chassis pose → baseline geometry solve → contact query/travel search → steering application → wheel hub state → Wheel contact/suspension update → tire state/force → chassis force application → wheel rotation integration. The rigid-body world step occurs outside Car.
  - Contact is queried once to select suspension travel and again inside Wheel::Update() after the steering-adjusted hub state is applied.
  - The first query's travel selection uses only local Y error, while the second query supplies the contact used for force/slip calculations.
  - Car has no MacPherson runtime selection/configuration path; vehicle configuration exposes Double Wishbone arm and spring-mount scalar parameters.
- **Evidence:** Vehicle/Car.cpp; Vehicle/Car.h; Vehicle/VehicleConfig.h; Vehicle/Wheel.cpp.
- **Assessment:** Double Wishbone is integrated, but the contact/travel coupling is only partially coherent and MacPherson is not integrated into the vehicle runtime.
- **Proposed remedy:** Define one authoritative contact/travel solve contract and avoid silently mixing query results from different hub poses. Introduce geometry selection/configuration only after the geometry API and validation requirements are agreed.
- **Agreed decision:** Pending user review.
- **Affected files:** To be determined after audit.
- **Verification:** VehicleDiagnostics passed in the recorded CI run, but does not establish MacPherson integration or full 3D contact constraint correctness.

### 5.7 Verification inventory — preliminary

- **Observed facts:**
  - The recorded GitHub Actions run for audited commit b247b5f2affd639cbaa69bea45f6eee1bb9e251a configured and built PhysicsDiagnostics and VehicleDiagnostics successfully.
  - VehicleDiagnostics exited 0 and reported [PASS] All vehicle diagnostics passed.
  - PhysicsDiagnostics exited 1 with [FAIL] Mechanical energy increased above initial energy.
  - The CI workflow does not build/run the standalone Double Wishbone, MacPherson, or Suspension diagnostics because their CMake options remain disabled in the workflow.
  - Source comments in the architecture document cite historical successful clean builds/ctest runs, but those are not fresh verification for this audited commit.
- **Evidence:** https://github.com/1hwai/DriveTest/actions/runs/37930118934; .github/workflows/physics-diagnostics.yml; CMakeLists.txt; diagnostic source files.
- **Assessment:** Verification is failed overall on the audited revision. The physics energy regression is real evidence of a failed test, but its root cause has not yet been established and must not automatically be attributed to suspension.
- **Proposed remedy:** Preserve the failing run as a blocker; inspect its energy snapshots and the exact energy-accounting assumptions before changing physics. Separately enable and run the focused geometry/suspension diagnostics on the audited revision.
- **Agreed decision:** Pending user review.
- **Affected files:** To be determined after root-cause isolation.
- **Verification:** Freshness is established by the recorded CI run for the audited commit; full root-cause diagnosis remains pending.

### Initial audit priorities (not yet approved)

1. Reproduce and isolate the mechanical-energy diagnostic failure without changing code.
2. Add/execute current-revision tests for the actual Double Wishbone and MacPherson solver invariants; explicitly distinguish solver validity from runtime integration.
3. Resolve the UpdateFromMounts() versus mechanical-limit contract.
4. Define contact-to-travel solve semantics and test full 3D residuals on non-flat surfaces.
5. Redesign the Car/Wheel/Suspension responsibility boundary based on the preceding evidence.

These are proposed priorities only; implementation remains blocked pending review and scope agreement.

### Historical work-unit traceability — source-level mapping

| Historical work unit | Source evidence found | Audit assessment on the audited revision |
|---|---|---|
| 1-1 Double Wishbone API/data model | DoubleWishbone.h/.cpp define arm, upright, hub, and spring mount state. | Implemented at source/API level; runtime correctness assessed separately. |
| 1-2 Double Wishbone solver | SolveArmAtHeight and SolveConstraints preserve configured link lengths and upright joint spacing. | Solver exists; independent current-revision run not established. |
| 1-2-4 Vehicle integration | Car owns DoubleWishbone[4] and sends solved hub state to Wheel. | Partially implemented: Wheel::Update still takes concrete DoubleWishbone and owns suspension/contact work. |
| 1-3 Travel | Car estimates feasible travel with 24 incremental samples each direction and searches travel by local hub Y. | Partially implemented; full 3D contact residual and boundary accuracy unverified. |
| 1-4 Double Wishbone validation | Focused test source checks arm lengths, upright distance, symmetry, steering-axis rotation, and finite states. | Test source exists, but current CI does not run it. Its tested scope does not prove all listed edge cases. |
| 1-M / 1-1 MacPherson API | MacPherson.h/.cpp define lower arm, strut, upright, hub, steering axis, and spring mounts. | Implemented as a standalone geometry class. |
| 1-M / 1-2 solver | MacPherson solver source and a focused diagnostic source exist. | Current-revision verification not established; diagnostic is not enabled in CI. |
| 1-M / 1-3 travel | SolveAtTravel changes target strut length and solves lower-arm/strut geometry. | Source exists; continuity/constraint claims require a fresh run and review of diagnostic independence. |
| 1-M / 1-4 final geometry validation | MacPhersonDiagnostics.cpp contains travel and geometry checks. | Historical checklist says passed, but current CI does not rebuild/run it. MacPherson is not connected to Car. |
| 2-1 responsibility/data flow | Architecture describes geometry-owned mounts and Suspension-owned scalar force model. | Design specified; active Wheel call path does not respect the intended boundary cleanly. |
| 2-2 spring/damper model | Suspension tracks mount distance, compression, compression velocity, and force magnitude. | Partially implemented: active UpdateFromMounts path bypasses ClampLength despite the documented contract. |
| 2-3 geometry/force connection | Wheel passes Double Wishbone spring mounts into Suspension and applies the resulting force to chassis mount A. | Active for Double Wishbone only; MacPherson mount output is not connected to Car. |
| 2-4 suspension validation | SuspensionDiagnostics.cpp checks scalar limits, damping selection, mount-derived force direction, and degenerate mounts. | Test source exists; current CI does not run it. |
| 3-1 steering axis | DoubleWishbone derives a world-space axis from upper/lower outer joints; MacPherson has its own axis methods. | Implemented for Double Wishbone runtime; MacPherson not integrated. |
| 3-2 steering geometry | Hub rotates around the computed axis; Ackermann angles are calculated from wheelbase and track. | Partial: config cannot express separate upper/lower outer-joint Z offsets or a general hub offset, and no physical steering-arm geometry is configured. Thus caster/scrub/steering-arm claims exceed current config expressiveness. |
| 3-3 steering/runtime link | Car applies front steering to DoubleWishbone geometry and propagates hub state to Wheel. | Implemented for the current Double Wishbone path; Wheel still stores and inverts a separate steering angle, so sign/data ownership remains split across two representations. |
| 3-4 Ackermann | CalculateAckermannAngles uses wheelbase and track width and applies inner/outer angles; VehicleDiagnostics checks turn direction and angle ordering. | Basic ideal Ackermann calculation implemented; not a mechanically modelled rack/tie-rod/steering-arm system. Vehicle test passed in recorded CI. |
| 4-1 tire interface | Tire consumes Wheel state and Car applies tire forces at Wheel contact point. | Core path implemented; the full suspension boundary remains coupled through Wheel. |
| 4-2 contact basis/force basis | Tire basis projects solved hub forward onto contact plane; VehicleDiagnostics checks basis alignment and force direction. | Current vehicle diagnostics passed in recorded CI; slope/edge/multiple-contact accuracy remains unverified. |
| 4-3 replaceable contact architecture | IWheelContactProvider is injected into Car; raycast provider implements it. | Provider boundary implemented, but current consumers only use the first sample and default query is a single world-down ray. |

### Additional source-level findings

- **Ignored solver failures:** DoubleWishbone::Solve() returns void and discards the result of SolveAtTravel(). Car::UpdatePhysics() also ignores the return from its baseline SolveAtTravel(..., 0). Failed geometry solves can therefore continue through the runtime without an explicit failure state.
- **Feasible travel is sampled, not solved to a boundary:** Car::ApplyConfig() checks 24 discrete travel samples in each direction and stops at the first failure. The stored limit is resolution-dependent rather than a refined feasibility boundary.
- **Steering configurability is narrower than the checklist implies:** VehicleConfig defines upper/lower outer joint X and Y but uses wheelZ for both outer-joint Z coordinates; hubOffset is vertical-only. Current config cannot independently set caster through fore-aft steering-axis offset or general lateral/fore-aft hub offset. Ackermann is calculated from wheelbase/track rather than a configured steering arm.
- **The current CI failure is not localized yet:** the job proves PhysicsDiagnostics fails its total mechanical-energy assertion, but the summarized output alone does not identify which force/state term introduces the increase. Do not attribute this to a specific suspension line until the saved energy snapshots and accounting are inspected.

## Phase 5.7 follow-up — energy test design and baseline correction

**Updated source revision:** 3e303dc21389edf8201209f249c38e1ad157afba  
**Scope:** Test/diagnostic changes only. No runtime physics implementation was changed.  
**Execution status:** The updated test has not yet been built or executed in this environment; the existing CI workflow only runs automatically on pushes/PRs to `main` or by manual dispatch. Do not mark this change verified until it runs.

### Evidence-based diagnosis of the previous failure

- `Car::ApplyConfig()` does call `Suspension::UpdateFromMounts(..., 0)`, so the compression scalar is not necessarily uninitialized.
- However, it initializes the geometry at its baseline travel. On the first regular `Car::UpdatePhysics()`, ground contact selects a different suspension travel and the spring mount distance/compression changes before the rigid-body integration step.
- `Tests/PhysicsDiagnostics.cpp` previously measured `initialEnergy` before this contact-consistent travel selection. Its initial spring energy therefore represented the baseline geometry, not the geometry that was about to be simulated.
- The CI artifact shows the first captured frame already has approximately 798.64 J of spring potential energy and reports a first-frame energy exceed. It also reports a first-frame suspension residual of approximately 350.52 W, then later residual around 11.94 W at the peak-energy sample. This supports an initialization/kinematic state-transition issue in the test setup, but does not prove that all later energy residuals are correct.
- **Revised assessment:** The previous CI failure is valid evidence that the diagnostic failed, but it was not sufficient evidence that the integrator generated energy during normal evolution. The test baseline was not aligned with the contact-selected suspension state.

### Test changes made

1. **Contact-consistent initial state:** `PhysicsDiagnostics` now performs a zero-time vehicle update before measuring initial energy. This lets the contact/travel path establish the suspension mount state without advancing rigid-body time. Accumulated forces from this initialization pass are cleared before the timed simulation begins.
2. **Absolute + relative tolerance:** The suspension stability guard now uses `max(2.0 J, 0.001 * abs(E0))`. At the current energy scale this is roughly 0.1%, rather than a universal 1 J threshold. This is an initial diagnostic guardrail, not a claim of a mathematically universal accuracy bound.
3. **Isolated timestep-convergence case:** Added a free-fall test using the existing rigid-body integrator at 1/60 s and 1/120 s. It checks that halving the timestep reduces the absolute mechanical-energy error by a clear margin. This test is not expected to conserve energy exactly: the current semi-implicit Euler update has timestep-dependent energy error.
4. **Explicit diagnostics:** Logs the free-fall error at each timestep and the suspension test's initial energy/tolerance. The existing peak/first-exceed per-wheel snapshots remain available for localization.

### Test matrix and interpretation

| Case | Active physics | Expected result | Failure points toward |
|---|---|---|---|
| Isolated free fall, dt=1/60 | Uniform gravity only; no contact, suspension, tire, or damping | Finite energy error | Integrator/state update or energy-accounting defect |
| Isolated free fall, dt=1/120 | Same initial state and duration as coarse case | Absolute error materially lower than coarse case | Timestep refinement does not improve the numerical solution |
| Contact-consistent suspension settle | Flat terrain, tire forces disabled, suspension spring/damper active | No non-finite state; no energy peak above initialized baseline by more than the absolute/relative tolerance; no lateral drift beyond existing bound | Initialization discontinuity, damping/force sign, spring-energy accounting, or geometry/contact coupling |
| Future: conservative spring/mass oscillator | One degree of freedom, no damping/contact | Bounded energy error that shrinks under timestep refinement | Spring force sign, integration, or spring potential calculation |
| Future: damper-only decay | One degree of freedom with positive damping | Mechanical energy is non-increasing within numerical tolerance | Damping sign or work/energy bookkeeping |
| Future: suspension travel sweep | Fixed chassis poses and sampled travel, both geometry families | Link constraints remain within geometric tolerances; no unexpected energy jump when comparing adjacent states | Kinematic solver discontinuity or mount/force coupling |

### Tolerance policy

- Use both absolute and relative tolerance: `tol = max(absTol, relTol * abs(E0))`.
- Keep the tolerance attached to the named test and its energy scale; do not copy a single threshold across tests with different units or magnitudes.
- For conservative numerical integration, prefer timestep-refinement/convergence checks in addition to a fixed tolerance. A single passing energy threshold can hide systematic drift.
- For damped or frictional systems, test the energy balance including dissipated work. Do not assert exact conservation of mechanical energy when non-conservative forces are active.
- Record total energy, potential/kinetic/spring components, peak energy, timestep, contact state, compression velocity, spring/damper force, suspension power, and residual. These values should be sampled from the same simulation step/state.

### Relevant external references

- [ODE Manual — numerical integration and energy growth](https://www.ode.org/wiki/index.php?title=Manual): describes how integration error and timestep affect rotational stability and energy growth in a rigid-body engine.
- [ODE Manual — joint error and ERP](https://ode.org/wiki/index.php/Manual): explains that constraint correction is approximate and joint alignment errors can remain; useful context for separating constraint error from energy error.
- [AscentBench validation — isolated two-body energy test](https://ascentbench.info/validation): illustrates isolating a conservative system, defining a duration/timestep, measuring maximum energy drift, and treating a stated percentage as a project-specific guardrail rather than a universal constant.
- [Energy-based monitoring of explicit co-simulation, Springer](https://link.springer.com/article/10.1007/s11044-022-09812-5): formulates energy balance as total mechanical energy minus initial energy minus work by non-conservative forces, supporting explicit work/residual tracking rather than checking total energy alone.
- [Kane, Marsden & Ortiz (1999), CaltechAUTHORS](https://authors.library.caltech.edu/records/hhgqy-wmf82): discusses energy/momentum-preserving variational integrators and shows that conservation properties depend on the numerical integration method.

### Remaining verification work

- Run the updated `DriveTestPhysicsDiagnostics` and inspect whether the new baseline removes the first-frame false exceed.
- Check the `[EnergyConvergence]` values and confirm the fine-step error is lower than the coarse-step error.
- If the suspension test still fails, compare `[EnergyPeak]`, `[EnergyFirstExceed]`, and each wheel's `suspensionResidual` after the initialized baseline. Then vary only one factor at a time: damping on/off, timestep, and contact/travel selection.
- Do not relax the tolerance merely to make CI green. If it fails, classify whether the failure is expected discretization error, a diagnostic accounting defect, or a runtime physics defect before changing the threshold.


## 2026-10-10 — Phase 6 work unit MP-KIN-01: constraint checks during chassis pose sweep

- **Work unit ID:** MP-KIN-01
- **Problem / observed facts:** The existing roll/pitch pose sweep in `Tests/MacPhersonDiagnostics.cpp` checks hub-offset magnitude, left/right symmetry, and finite state, but does not directly verify the lower-arm link lengths and strut length in rotated chassis poses. The file already defines a `ToWorld()` helper, but it was not used for these constraint checks.
- **Evidence:** The `poseSweep` loop and `ToWorld()` in `Tests/MacPhersonDiagnostics.cpp`; related implementation in `Vehicle/MacPherson.cpp` (`SolveAtTravel()`, `SolveConstraints()`, `UpdateUpright()`).
- **Interpretation / confidence:** Missing checks for core length invariants in rotated poses are a test-coverage gap. This alone does not establish that the solver implementation is incorrect.
- **Proposed remedy:** For each successful sample in the existing roll/pitch sweep, transform local pivots and the upper strut mount into world space and verify both lower-arm link lengths and the distance from the upper mount to the lower strut mount against their configured lengths.
- **Agreed decision:** Narrowly address this diagnostic gap within the MacPherson-first scope. Do not modify Double Wishbone, vehicle runtime, or spring-force behavior in this work unit.
- **Allowed files:** `Tests/MacPhersonDiagnostics.cpp`.
- **Actual files changed:** `Tests/MacPhersonDiagnostics.cpp`.
- **Expected result / invariants:** At every successful sampled roll/pitch pose, both arm lengths and the strut length must remain within tolerance. A violation must cause the diagnostic to return a non-zero exit code.
- **Verification commands:** `cmake -S . -B build -DDRIVETEST_BUILD_APP=OFF -DDRIVETEST_BUILD_MACPHERSON_DIAGNOSTICS=ON`; `cmake --build build --target DriveTestMacPhersonDiagnostics`; `./build/DriveTestMacPhersonDiagnostics`.
- **Verification result:** **Unverified.** The source change was committed, but this work did not build or execute the diagnostic. Do not report a passing result until command output is available.
- **Status:** Test code changed / execution verification pending.
- **Revision:** `dc4b08465b9f5ce63a183beb1f4279b6c03ff5d6`.
- **Remaining limits / dependency:** The new checks cover only the existing finite set of roll/pitch angles at zero travel. They do not prove continuity over the full stroke, behavior at failure boundaries, assembly-branch uniqueness, or vehicle runtime integration. Next, build and run the diagnostic at this revision and record the result.


### MP-KIN-01 execution result — 2026-10-10

- **User-provided output:** `[MacPherson] solved=1 armLengths=1 strutConstraint=1 symmetry=1 hubPosition=1 steeringAxis=1 zeroSteering=1 steeringRotation=1 finite=1 travel=1 endpoints=1 poseSweep=1 poseConstraints=1 continuity=1 maxHubStep=0.0265927 maxHorizontalDrift=0.0213085`; final status `[PASS] MacPherson travel diagnostics`.
- **Assessment:** The new pose-sweep length-constraint checks in MP-KIN-01 passed. Every existing diagnostic flag was also 1 in this run.
- **Scope:** The finite roll/pitch pose samples and travel samples currently included in the diagnostic. This does not establish continuity over the full stroke, behavior at failure boundaries, uniqueness of all assembly branches, or vehicle runtime integration.
- **Evidence limitation:** This result is based on terminal output supplied by the user; no separate build or execution was performed during this documentation update.


## 2026-10-10 — MP-KIN-02: reject non-finite travel input

- `MacPherson::SolveAtTravel()` now checks whether `travel` is finite and rejects NaN or infinity before updating geometric state.
- `MacPhersonDiagnostics` checks that NaN and positive infinity return `false` and leave the previous hub position unchanged.
- Change revisions: `12803ae27eb2f2c78cc783af18e93a7400e4a9b7` (diagnostic), `668036988150ee3fd73f0188edbbb02627151441` (solver).
- Verification status: source changes are committed; build and execution are pending. Passing status has not yet been confirmed.
- Limit: this change validates only the `travel` argument; it does not validate every geometry configuration value.


### MP-KIN-02 execution result — 2026-10-10

- **User-provided output:** The build linked `DriveTestMacPhersonDiagnostics`. The diagnostic output included `invalidTravelRejected=1`, all reported flags were `1`, and the final line was `[PASS] MacPherson travel diagnostics`.
- **Assessment:** The checks rejecting NaN and positive-infinity travel inputs, while preserving the prior hub position, passed.
- **Scope:** This is based on the supplied local build/run output. It establishes handling of non-finite `travel` input only; it does not establish validation of all configuration values, full-stroke continuity, or vehicle runtime integration.
- **Evidence limitation:** The user supplied the execution output; no separate build or execution was performed during this documentation update.
- **Status:** MP-KIN-02 verified (scope-limited).


## 2026-10-10 — MP-KIN-03: preserve geometry when a solver call fails

- **Observed issue:** MP-KIN-02 rejects NaN and infinity before changing state, but a finite travel value can still fail because the target strut length is invalid or the geometric constraints cannot be solved. Previously, `SolveAtTravel()` updated pivots and mounts before those failure checks, potentially leaving partially updated geometry even though it returned `false`.
- **Change:** `SolveAtTravel()` snapshots the lower-arm, strut, and upright states and restores all three if the strut-length check or `SolveConstraints()` fails.
- **Diagnostic:** Tests rejection of NaN, positive infinity, and a finite but impossible travel (`travel == GetStrutLength()`). After each rejection, it checks that hub position, lower outer joint, spring mounts A/B, and hub orientation remain unchanged.
- **Change revisions:** `157bf2a3d33a3c643711fcb97c967e4ad8f046db` (solver), `6ee56ccb07ad1e2b14688ac7fa7ac5419064c9d5` (diagnostic).
- **Verification status:** Source changes are committed. Awaiting the user's local build/run output; not yet marked as passing.
- **Remaining limits:** The rollback covers strut-length and geometric-constraint failures in `SolveAtTravel()`. Validation of invalid values passed to `Configure()` and vehicle runtime integration are outside this work unit.
- **Verification commands:** `git pull --rebase origin suspension-runtime-integration`; `cmake --build build --target DriveTestMacPhersonDiagnostics`; `./build/DriveTestMacPhersonDiagnostics`.


### MP-KIN-03 execution result — 2026-10-10

- **User-provided output:** `DriveTestMacPhersonDiagnostics` linked successfully. The output included `invalidTravelRejected=1`, every reported diagnostic flag was `1`, and the final line was `[PASS] MacPherson travel diagnostics`.
- **Assessment:** Rejection of NaN, positive infinity, and finite but impossible travel values passed, as did preservation of the checked geometry state after rejection.
- **Scope:** The diagnostic checks preservation of hub position, lower outer joint, spring mounts A/B, and hub orientation. It does not establish full-stroke continuity, validation of all `Configure()` inputs, or vehicle runtime integration.
- **Evidence limitation:** The user supplied the execution output; no separate build or execution was performed during this documentation update.
- **Status:** MP-KIN-03 verified (scope-limited).


## 2026-10-10 — MP-KIN-04: increase travel-continuity sample density

- **Rationale:** The existing diagnostic compared adjacent hub positions at only seven samples from `-0.075` to `+0.075`. That spacing can miss local discontinuities, so the sample density is increased within the currently tested range.
- **Change:** The travel sweep in `Tests/MacPhersonDiagnostics.cpp` now uses 31 samples at 0.005 intervals. The horizontal-drift reference is captured from an unsteered zero-travel solve. The maximum adjacent hub-step continuity threshold is tightened to `0.01`.
- **Allowed file:** `Tests/MacPhersonDiagnostics.cpp` only.
- **Excluded:** Solver implementation, geometry configuration, Double Wishbone, runtime integration, and expansion of the tested travel range.
- **Change revision:** `c4b96318a04b268ff7edc26ada693fc880739607`.
- **Verification status:** Source change committed; awaiting the user's local build/run output.
- **Limit:** 31 samples provide a continuity metric over the specified `[-0.075, +0.075]` range, not a mathematical proof for every intermediate state or exhaustive exploration of solver-failure boundaries.
- **Verification commands:** `git pull --rebase origin suspension-runtime-integration`; `cmake --build build --target DriveTestMacPhersonDiagnostics`; `./build/DriveTestMacPhersonDiagnostics`.


### MP-KIN-04 execution result — 2026-10-10

- **User-provided output:** The `DriveTestMacPhersonDiagnostics` output showed all diagnostic flags as `1`, with `maxHubStep=0.00539565`, `maxHorizontalDrift=0.0213085`, and the final line `[PASS] MacPherson travel diagnostics`.
- **Assessment:** The tightened travel-continuity threshold (`maxHubStep < 0.01`) passed. The observed maximum adjacent hub step was approximately `0.00540`.
- **Scope:** 31 samples over the specified `[-0.075, +0.075]` travel interval. This does not guarantee continuity at every intermediate state or across other vehicle configurations.
- **Evidence limitation:** The user supplied the execution output; no separate build or execution was performed during this documentation update.
- **Status:** MP-KIN-04 verified (scope-limited).


### MP-KIN-04 execution result — 2026-10-10

- **User-provided output:** All diagnostic flags were `1`; `maxHubStep=0.00539565`, `maxHorizontalDrift=0.0213085`; final line `[PASS] MacPherson travel diagnostics`.
- **Assessment:** The tightened `maxHubStep < 0.01` threshold passed across 31 travel samples.
- **Limit:** Samples cover only `[-0.075, +0.075]`; they do not guarantee every intermediate state or other configurations. Output was supplied by the user; no separate execution was performed during this documentation update.
- **Status:** MP-KIN-04 verified (scope-limited).


## 2026-10-10 — MP-KIN-05: validate chassis position and orientation inputs

- **Problem:** MacPherson::SolveAtTravel() validated only travel finiteness. A non-finite chassis position or a non-finite, zero, or non-unit orientation quaternion could propagate invalid values into geometry calculations.
- **Change:** Before mutating geometry, validate that all chassis position components are finite and that all quaternion components and its squared norm are finite. The quaternion squared norm must be within 0.001 of 1. Invalid inputs return false; the input quaternion is not silently normalized.
- **Diagnostic:** Reject NaN/infinite position components, NaN/infinite quaternion components, a zero quaternion, and a non-unit quaternion. Verify rejection preserves hub position, lower outer joint, spring mounts A/B, and hub orientation.
- **Change revisions:** solver b247a861d1085077dfaf70b32019420404811679; diagnostic ed8440441ff77df51104cf401b8afbcda010c98a.
- **Verification status:** Source and diagnostic changes committed. Awaiting local build/run output; not yet marked passing.
- **Limit:** This validates the chassis pose passed to SolveAtTravel() only. Validation of all Configure() values, quaternion construction paths, and vehicle runtime integration are out of scope.
- **Verification commands:** git pull --rebase origin suspension-runtime-integration; cmake --build build --target DriveTestMacPhersonDiagnostics; ./build/DriveTestMacPhersonDiagnostics.


### MP-KIN-05 execution result — 2026-10-10

- **User-provided output:** `nonFinitePositionRejected=1`, `invalidOrientationRejected=1`, `invalidChassisPoseRejected=1`, and all existing diagnostic flags were `1`; final line `[PASS] MacPherson travel diagnostics`.
- **Assessment:** Rejection of non-finite chassis positions and invalid orientation quaternions, plus preservation of the checked geometry state after rejection, passed.
- **Scope:** Limited to the invalid input cases explicitly included in the diagnostic. This does not validate every `Configure()` value or vehicle runtime integration.
- **Evidence limitation:** The user supplied the execution output; no separate build or execution was performed during this documentation update.
- **Status:** MP-KIN-05 verified (scope-limited).


## 2026-10-10 — MP-KIN-06: test constraints across translated chassis poses and nonzero travel

- **Problem / evidence:** The existing pose sweep checked roll/pitch only at zero travel and zero chassis position. The separate travel sweep covered `[-0.075, +0.075]` only with identity chassis orientation. Combined nonzero chassis translation, rotation, and suspension travel were therefore not covered.
- **Agreed scope:** Extend `Tests/MacPhersonDiagnostics.cpp` only. Leave the solver, configuration, Double Wishbone, spring-force model, and runtime integration unchanged.
- **Change:** The pose sweep now uses a nonzero chassis position `(3, -2, 4)`, the existing roll/pitch grid, and travel samples `-0.05, 0, +0.05`. For each successful left/right solve, it checks both lower-arm link lengths, expected strut length (`configured length - travel`), lower-mount and hub offsets, finite/unit hub orientation, and mirrored left/right hub positions in chassis-local space.
- **Expected result:** Every sampled combination solves and satisfies the geometric invariants within diagnostic tolerance.
- **Allowed / changed files:** `Tests/MacPhersonDiagnostics.cpp` only.
- **Change revision:** `ffb1692e7e2d2822dbf21e32b20ae1a744c1076a`.
- **Verification commands:** `git pull --rebase origin suspension-runtime-integration`; `cmake --build build --target DriveTestMacPhersonDiagnostics`; `./build/DriveTestMacPhersonDiagnostics`.
- **Verification status:** **Pending user build/run output.** The source update is committed, but the diagnostic was not built or executed during this work. Do not mark the expanded pose sweep as passing until its output is inspected.
- **Limits:** This adds combined-pose samples; it is not exhaustive coverage of every travel value, configuration, solver failure boundary, or assembly branch. It does not integrate MacPherson into vehicle runtime.


### MP-KIN-06 execution result — 2026-10-10

- **User-provided output:** All existing diagnostic flags, `poseSweep=1`, and `poseConstraints=1` passed; final output was `[PASS] MacPherson travel diagnostics`. `maxHubStep=0.00539565`, `maxHorizontalDrift=0.0213085`.
- **Assessment:** The diagnostic passed for the translated chassis position, roll/pitch poses, and three travel samples.
- **Interpretation note:** `maxHorizontalDrift` is the maximum absolute x/z coordinate deviation from the reference hub position over the travel sweep; it is neither the magnitude of the horizontal displacement vector nor pure numerical error. Its threshold is 0.20m, and this result alone does not establish geometric accuracy against a real vehicle.
- **Evidence limitation:** The user supplied the build/run output; no separate build or execution was performed during this documentation update.
- **Status:** MP-KIN-06 verified (diagnostic scope only).


## 2026-10-10 — MP-KIN-07: test geometric-constraint failure and rollback

- **Observed facts:** Existing failure diagnostics covered NaN/infinite inputs and the early invalid-strut-length guard, but did not directly exercise a finite travel value with a positive target strut length for which the lower-arm/strut geometric intersection is infeasible.
- **Evidence:** In `Vehicle/MacPherson.cpp`, `SolveAtTravel()` checks strut length before calling `SolveConstraints()`; an infeasible intersection returns `false` and restores the saved state. For the diagnostic configuration, `travel = 0.30f` was selected to keep the target length positive while placing the target outside the feasible intersection range.
- **Interpretation:** This checks the geometric-constraint failure path and state rollback. It does not cover every singularity or every configuration.
- **Agreed scope:** Add a regression check to `Tests/MacPhersonDiagnostics.cpp`. Do not change the solver algorithm, configuration API, Double Wishbone, or vehicle runtime.
- **Change:** `unreachableConstraintRejected` verifies rejection and preservation of hub position, lower outer joint, spring mounts A/B, and hub orientation; failure causes the diagnostic executable to fail.
- **Verification commands:** `git pull --rebase origin suspension-runtime-integration`; `cmake --build build --target DriveTestMacPhersonDiagnostics`; `./build/DriveTestMacPhersonDiagnostics`.
- **Verification status:** Awaiting the user's local build/run output after commit. No build or execution was performed during this documentation update.
- **Limits and next step:** This covers one deliberately infeasible case for the synthetic test configuration. Next, inspect assembly-branch selection between the two intersection candidates and continuity near singular configurations.
