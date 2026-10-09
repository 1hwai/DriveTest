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
