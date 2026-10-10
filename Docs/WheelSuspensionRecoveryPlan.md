# Wheel & Suspension Recovery Workflow

> This document adds a follow-on workflow for auditing and correcting the current implementation. It does **not** replace or renumber the historical work in `Docs/WheelSuspensionArchitecture.md`.
>
> The existing architecture document remains the design source of truth. This workflow records what the current code actually does, resolves discrepancies, and defines the work needed to reach that design.
>
> Keep this English document and `Docs/WheelSuspensionRecoveryPlan.ko.md` synchronized. Record evidence-based findings and agreed solutions in the paired `.solution.md` documents.

## Why this workflow exists

The original work units `1-1` through `4-2` established the intended geometry, suspension, steering, and tire-integration direction. However, the presence of classes or a successful call path does not prove that the mechanical model is correct or that the runtime uses its results consistently.

The follow-on work must address these known questions without assuming their causes in advance:

- Do Double Wishbone and MacPherson solve their intended mechanical constraints, rather than approximating wheel travel with simplified motion?
- Are spring/damper mounting points, lengths, axes, forces, and reaction forces mechanically consistent?
- Are `Car`, `Wheel`, `Suspension`, geometry, steering, and tire responsibilities separated without duplicate or contradictory calculations?
- Can wheel-ground contact be queried through a replaceable boundary, without coupling `Wheel` to one contact algorithm?
- Does the integrated car behave consistently, including during roll, pitch, airborne motion, contact loss, and rollover?

## Workflow rules

1. **Audit before modifying implementation.** Phase 5 is read-only: inspect code, docs, tests, diagnostics, and runtime evidence. Do not refactor during the audit.
2. **Trace data and forces end to end.** Record the source, coordinate space, owner, consumer, and update order for important values.
3. **Separate implementation status from verification status.** An existing class or function is not proof of correct behavior.
4. **Use evidence, not visual guesses.** Prefer source tracing, independent geometric invariants, focused diagnostics, tests, and fresh runtime checks.
5. **Do not silently rewrite history.** Preserve the original `1-1` through `4-2` work history. Any old completion claim found to be unsupported should be recorded in the audit and then corrected explicitly in the original document when the scope is agreed.
6. **Agree scope before each implementation work unit.** List the goal, exact files allowed to change, excluded files, expected result, and verification command/test.
7. **Keep the solution log current.** At the end of each phase, add findings, evidence, the agreed solution, affected files, verification, and unresolved questions to `WheelSuspensionRecoveryPlan.solution.md` and its Korean counterpart.
8. **Do not claim completion without fresh evidence.** Build success, test success, and physical correctness are distinct claims.

## Status vocabulary

Each requirement gets two separate fields: implementation assessment and verification assessment.

### Implementation assessment

- **Implemented** — source and data flow appear to satisfy the requirement.
- **Partially implemented** — some required behavior exists, but important parts are missing.
- **Incorrectly connected** — code exists, but ownership, coordinate spaces, update order, or data/force flow conflicts with the design.
- **Not implemented** — no implementation was found for the requirement.
- **Unknown** — the relevant implementation or call path has not yet been located.

### Verification assessment

- **Verified** — suitable, current evidence directly supports the stated invariant.
- **Failed** — a reproducible check contradicts the requirement.
- **Not verified** — evidence is absent, stale, indirect, or insufficient.

Do not collapse these into a single status. For example, an implemented contact-provider interface can still have unverified contact-point accuracy.

---

# Phase 5 — Implementation Audit

**Goal:** Establish an evidence-backed picture of what exists, how it is connected, and where it diverges from the architecture.

**Implementation changes:** None during Phase 5.

## 5.1 Requirements traceability

- [ ] Extract requirements and completion claims from the existing architecture document, including `1-1` through `4-2`.
- [ ] Link each requirement to concrete files, classes, functions, and call sites.
- [ ] Inspect checklist marks rather than treating them as proof.
- [ ] Record implementation status and verification status separately.
- [ ] Preserve historical numbering and clearly distinguish old work units from this recovery workflow.

**Deliverable:** A requirement-to-code traceability table with evidence and gaps.

## 5.2 Kinematic/mechanical correctness

- [ ] Locate the concrete Double Wishbone and MacPherson implementations and all runtime call sites.
- [ ] Trace the geometry inputs: pivots, link/joint positions, strut mounts, hub offsets, steering-axis data, and coordinate spaces.
- [ ] Determine whether each solver enforces the intended fixed-length/joint constraints.
- [ ] Check whether hub position and orientation are derived from solved geometry or partly replaced by simplified corrections.
- [ ] Check bump/rebound limits, solver failure/fallback behavior, continuity, left/right symmetry, and feasible-travel calculations.
- [ ] Identify any wheel movement derived from a fixed world-down vector, a chassis-local vertical offset, or another approximation that bypasses the suspension constraints.
- [ ] Check whether MacPherson is independently implemented and actually used at runtime; do not infer this from configuration types or declarations alone.

**Deliverable:** Per-geometry constraint and runtime-usage assessment, with specific violations or missing evidence.

## 5.3 Spring/damper and force path

- [ ] Trace spring/damper mount points from geometry configuration to world-space positions.
- [ ] Confirm that spring length and axis are derived from the two actual mounts.
- [ ] Trace compression, compression velocity, spring force, compression/rebound damping, and travel limits.
- [ ] Trace where the force is applied, at what point and direction, and how the equal-and-opposite reaction is represented.
- [ ] Determine whether the force is transmitted through the intended mechanical assembly or applied directly through a shortcut.
- [ ] Check grounded, airborne, fully compressed, fully extended, and rollover behavior for artificial attraction or invalid force direction.
- [ ] Check whether diagnostics use the current geometry or stale assumptions such as a fixed vertical suspension axis.

**Deliverable:** A force-flow trace from mount geometry through force calculation to chassis response, identifying every unexplained or inconsistent step.

## 5.4 Responsibility and data ownership

- [ ] Inventory responsibilities in `Car`, `Wheel`, `Suspension`, each geometry solver, steering, tire, and contact-query code.
- [ ] Trace ownership of hub position/orientation, suspension travel, spring mounts, contact state, wheel rotation, and tire-force state.
- [ ] Find duplicated calculations, stale state, hidden mutation, and values recomputed differently by different classes.
- [ ] Check whether `Wheel` depends on a concrete suspension implementation or performs geometry work that belongs elsewhere.
- [ ] Check that components exchange only the data needed by their responsibilities.
- [ ] Record the intended runtime update order and compare it with actual calls.

**Deliverable:** Responsibility matrix and current-state data-flow diagram.

## 5.5 Ground-contact boundary

- [ ] Locate the contact-provider/query interface and every implementation.
- [ ] Trace how wheel/hub state becomes a query and how the result is consumed by suspension and tire logic.
- [ ] Check the meaning and coordinate space of hit position, normal, distance, and validity.
- [ ] Check behavior for no hit, multiple candidate hits, terrain/road boundaries, and invalid normals.
- [ ] Determine whether the boundary permits alternate algorithms or combined queries without changing `Wheel`.
- [ ] Distinguish replaceability of the API from physical accuracy of the chosen algorithm.

**Deliverable:** Contact-query call graph, data contract, and coupling/accuracy findings.

## 5.6 Vehicle runtime integration

- [ ] Trace the actual order from chassis state through geometry, steering, hub state, contact, suspension, tire force, force application, and rigid-body integration.
- [ ] Inspect `Car.cpp/.h`, `Wheel.cpp/.h`, `Suspension.cpp/.h`, geometry classes, tire code, configuration, and relevant diagnostics.
- [ ] Check steering sign conventions and steering-axis use against the documented vehicle-local convention: forward +Z, up +Y, right -X, left +X.
- [ ] Identify legacy calculations that override, duplicate, or contradict the new geometry results.
- [ ] Check config-to-runtime transfer and default/fallback paths.
- [ ] Compare visible symptoms with telemetry and source evidence; do not assume every odd visual result is a rendering bug or every handling issue is a physics bug.

**Deliverable:** Runtime call sequence and a prioritized integration-defect list.

## 5.7 Verification inventory

- [ ] Inventory available build targets, unit tests, physics diagnostics, vehicle diagnostics, logs, and reproducible runtime scenarios.
- [ ] Identify which tests independently check geometric invariants and which only repeat implementation assumptions.
- [ ] Record commands, expected invariants, observed output, exit status, and revision tested.
- [ ] Identify missing tests for symmetry, constraint residuals, continuity, force direction, airborne motion, contact loss, rollover, and steering through suspension travel.
- [ ] Do not use compilation alone as evidence of physical correctness.

**Deliverable:** Verification matrix showing proven behavior, failed behavior, and unverified behavior.

## Phase 5 exit criteria

- [ ] Every requirement from the original `1-1` through `4-2` scope has a recorded assessment or an explicit reason it remains unknown.
- [ ] The main mechanical and integration issues are supported by source references, diagnostics, tests, or clearly labelled missing evidence.
- [ ] Findings are prioritized by physical correctness, architectural impact, and dependency.
- [ ] No implementation changes were made as part of the audit.
- [ ] The user has reviewed and agreed on the findings and proposed correction order.

---

# Phase 6 — Correct Kinematic Models

**Goal:** Make each suspension geometry determine mechanically constrained hub motion over its usable travel.

- [ ] Turn Phase 5 findings into narrowly scoped work units.
- [ ] Fix the Double Wishbone solver and its constraints where evidence requires it.
- [ ] Fix the MacPherson solver and its constraints where evidence requires it.
- [ ] Make travel bounds and infeasible configurations explicit; avoid silent fallbacks that create physically impossible states.
- [ ] Validate hub position/orientation, camber, track and wheelbase changes, symmetry, and travel continuity against independent geometric invariants.
- [ ] Ensure spring mounts follow the solved geometry.
- [ ] Record exact files and verification for each work unit before changing code.

**Exit criteria:** Each concrete model passes focused geometry tests independently. Runtime integration is not considered complete merely because the solver tests pass.

# Phase 7 — Responsibility and Interface Boundaries

**Goal:** Remove contradictory ownership and ensure components exchange only the physical data they need.

- [ ] Define ownership of suspension geometry, hub state, spring/damper state, contact state, wheel rotation, and tire forces.
- [ ] Ensure geometry solvers own kinematics rather than duplicating them in `Wheel` or `Car`.
- [ ] Ensure the suspension force model consumes the required mount/state data and returns explicit force results.
- [ ] Ensure `Wheel` consumes hub/contact/wheel state without depending on a concrete suspension algorithm.
- [ ] Keep tire-force calculation independent of the internal suspension implementation.
- [ ] Avoid introducing abstractions without a demonstrated responsibility or substitution need.

**Exit criteria:** A documented call/data-flow contract matches the code and can be tested at component boundaries.

# Phase 8 — Replaceable Ground-Contact Query

**Goal:** Decouple wheel/tire logic from the contact-search algorithm and define a stable result contract.

- [ ] Specify query input and result semantics, including coordinate spaces, hit validity, contact point, normal, and relevant distance/feature data.
- [ ] Make the wheel path consume the provider contract rather than a hard-coded raycast implementation.
- [ ] Keep query implementation(s) outside the wheel's suspension/force responsibilities.
- [ ] Define how alternate or multiple queries are selected/combined, if needed, without prematurely building a generic framework.
- [ ] Test terrain, road, edges, no-hit, invalid-hit, and contact-transition cases.
- [ ] Verify that contact constraints do not create artificial chassis attraction or override kinematic suspension limits.

**Exit criteria:** The contact algorithm can be substituted or extended at the boundary, and the result semantics are independently tested.

# Phase 9 — Vehicle Runtime Integration

**Goal:** Connect the validated geometry, spring/damper, steering, contact, tire, and rigid-body paths in one coherent update sequence.

- [ ] Document the actual intended update order and state dependencies.
- [ ] Remove stale or duplicate calculations that contradict the chosen ownership model.
- [ ] Ensure steering transforms the hub through the configured steering geometry.
- [ ] Ensure suspension forces and tire forces use the correct points, directions, and coordinate spaces.
- [ ] Ensure contact loss, airborne travel, and rollover do not inject invalid corrective forces.
- [ ] Confirm configuration and diagnostics describe the new runtime model.
- [ ] Test ordinary driving and uneven road contact before tuning handling parameters.

**Exit criteria:** Runtime behavior agrees with the component contracts and no known legacy path overrides the validated geometry.

# Phase 10 — Validation and Regression

**Goal:** Prove the integrated system against independent invariants and repeatable scenarios.

- [ ] Run clean build and focused geometry tests.
- [ ] Run force-direction, spring/damper, and contact-boundary tests.
- [ ] Run symmetry, full bump/rebound, continuity, and constraint-residual tests.
- [ ] Run chassis roll/pitch, one-wheel lift, airborne, contact loss/recovery, extreme steering, and rollover scenarios.
- [ ] Run repeatable vehicle diagnostics for steering direction, lateral load transfer, tire contact, and unwanted wheel drift.
- [ ] Test on representative terrain and road transitions.
- [ ] Record revision, commands, test output, failures, limitations, and any remaining physical approximations.
- [ ] Update the original architecture checkboxes only where fresh evidence supports them; keep this recovery plan and solution log synchronized.

**Exit criteria:** Each claimed behavior has fresh evidence. Known approximations and failures are explicit; untested cases remain marked unverified.

---

## Per-work-unit change contract

Before any implementation work unit after Phase 5, record:

- **Work-unit ID**
- **Problem and evidence**
- **Goal / observable success criteria**
- **Files allowed to change**
- **Files explicitly out of scope**
- **Expected result**
- **Verification command(s) and invariant(s)**
- **Rollback or fallback considerations**

After the work unit, record the diff summary and fresh verification results in the solution log. Do not expand scope silently.

## Companion documents

- Architecture specification: `WheelSuspensionArchitecture.md`
- Korean architecture specification: `WheelSuspensionArchitecture.ko.md`
- Korean recovery workflow: `WheelSuspensionRecoveryPlan.ko.md`
- Findings and solution log: `WheelSuspensionRecoveryPlan.solution.md`
- Korean findings and solution log: `WheelSuspensionRecoveryPlan.solution.ko.md`
