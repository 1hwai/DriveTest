# DriveTest Engineering Skill

## Purpose
Use this skill for DriveTest implementation, debugging, architecture changes, and repository edits.
The goal is not to produce code quickly. The goal is to make the smallest correct change, prove it works, and leave the repository easier to reason about.

## 1. Think Before Coding
Before changing code:
1. Restate the actual failure or goal in concrete terms.
2. Identify the relevant subsystem and its current ownership.
3. State assumptions that are not yet verified.
4. Consider at least two plausible interpretations when the problem is ambiguous.
5. Prefer the explanation supported by code, logs, tests, or reproducible behavior.
6. If the requested approach conflicts with observed architecture or evidence, say so and propose the correction.
7. Stop and inspect more code instead of inventing an explanation when evidence is insufficient.
Do not turn an unclear symptom into an implementation task prematurely.

## 2. Scope Before Editing
Every implementation work unit must have an explicit file scope.
Before editing, state:
- Work-unit ID.
- Goal.
- Exact files allowed to change.
- Files explicitly out of scope.
- Verification required to mark the work unit complete.
For DriveTest, follow the work-unit/checklist convention in the relevant architecture document.
Do not modify implementation code while an architecture/scope review is unresolved.

## 3. Surgical Changes
Change only what is necessary for the stated goal.
- Match existing project style.
- Do not refactor unrelated code.
- Do not add speculative abstractions.
- Do not rename APIs merely because another name seems nicer.
- Do not clean up adjacent code unless the change creates orphaned or invalid code.
- Preserve existing behavior outside the stated change.
- If a broader change is actually required, stop and explain why before expanding scope.
A smaller diff is preferred when it solves the same verified problem.

## 4. Goal-Driven Implementation
Translate implementation requests into observable success criteria.
Bad: Fix suspension.
Good:
- Wheel center must remain mechanically constrained during chassis roll.
- The wheel must not acquire artificial world-X displacement from suspension travel.
- The diagnostic must demonstrate the expected behavior.
Every multi-step plan should follow: Step -> Expected result -> Verification.
Do not mark a work unit complete merely because the code looks reasonable.

## 5. Evidence Over Explanation
When debugging, prefer this evidence order:
1. Reproduction.
2. Relevant source code.
3. Instrumented logs / state.
4. Focused diagnostic or test.
5. Full build/test result.
6. Runtime behavior.
A plausible explanation is not a verified explanation.
If evidence contradicts the current hypothesis, discard the hypothesis instead of defending it.

## 6. No Unverified Architecture Claims
Do not claim an architecture is implemented, correct, stable, or complete merely because classes/API exist.
For an architecture work unit, completion requires the applicable combination of:
- documented requirements satisfied;
- actual call/data ownership matching the design;
- relevant code compiling;
- focused diagnostics/tests passing;
- runtime behavior matching the stated success criteria.
Architecture documentation and implementation must not silently diverge.

## 7. Work-Unit Checklist Discipline
DriveTest architecture documents use [ ] / [x] checklists.
- Mark an item [x] only after the required evidence exists.
- If a completed item must be redesigned, remove [x] before revising it.
- Keep the document synchronized with actual implementation state.
- Do not use the checklist as a prediction of future success.

## 8. Debugging Discipline
When a bug persists:
- Do not repeat the same patch with minor variations without new evidence.
- Do not blame external systems without testing the hypothesis.
- Do not silently change the coordinate convention.
- Trace ownership and data flow from source to symptom.
- Check whether the diagnostic itself can reproduce a false positive/false negative.
- Prefer a minimal reproduction over adding more logging everywhere.
If a previous fix only made a test pass because the test duplicated the implementation's assumptions, explicitly invalidate that evidence and redesign the test.

## 9. Verification Before Completion
Before saying fixed, done, complete, passing, or equivalent:
1. Identify the command or diagnostic that proves the claim.
2. Run it fresh after the final code change.
3. Read the complete relevant output and exit status.
4. Check that the output actually proves the stated requirement.
5. If verification fails, do not claim completion.
6. Report the actual failure and next corrective step.
Do not substitute an earlier successful run, compilation for behavioral verification, a linter for a compiler/test, code review for runtime evidence, or 'should work' for evidence.

## 10. Git Discipline
Before committing:
- Confirm the diff contains only intended changes.
- Confirm the work-unit scope.
- Run required verification fresh.
- Commit only after verification supports the commit's claim.
Commit messages should describe the actual completed change, not an intended future state.

## 11. DriveTest-Specific Constraints
- Coordinate convention: Forward = +Z, Up = +Y, Right = -X, Left = +X.
- Treat GitHub as the source of truth for repository state.
- Prefer existing architecture and APIs unless the work unit explicitly replaces them.
- For wheel/suspension work, use Docs/WheelSuspensionArchitecture.md as the source of truth.
- Do not preserve obsolete suspension behavior merely for compatibility when the architecture explicitly replaces it.
- Do not introduce a general-purpose framework or solver when the documented work unit requires only the minimum necessary mechanism.

## 12. Response Discipline
When working with the user:
1. State the next concrete step.
2. State the exact files needed before requesting files.
3. After receiving files, modify only the agreed scope.
4. Give the verification result, including failures when present.
5. Never hide uncertainty behind confident wording.

The standard is:
Understand -> Scope -> Change -> Verify -> Report.