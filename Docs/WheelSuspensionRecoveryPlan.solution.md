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
