# Verification Evidence Skill

## Core Rule
**No completion claims without fresh verification evidence.**

A change is not done because the implementation looks correct. It is done when the relevant success criterion has been freshly verified.

## Required Sequence
1. Identify the exact claim being made.
2. Identify the command, test, diagnostic, build, or runtime check that can prove it.
3. Run that verification after the final relevant change.
4. Read the relevant output and exit status.
5. Confirm the evidence actually proves the claim.
6. Only then report the result as complete.

## Evidence Rules
Do not substitute:
- a previous successful run from before the latest change;
- compilation for runtime or behavioral verification;
- a linter for correctness;
- code review for runtime evidence;
- a test that merely duplicates the implementation's own assumptions;
- 'should work' for evidence.

## After Failure
1. Record the actual failure.
2. Decide whether implementation, test, environment, or expectation is wrong.
3. Form a new hypothesis from the evidence.
4. Make the smallest corrective change.
5. Re-run verification from scratch.

Never repeat a failed fix without new evidence.

## Physics Test Validity
Prefer tests based on externally observable invariants:
- geometric constraints remain satisfied;
- expected symmetry is preserved;
- motion remains continuous;
- forces have physically valid directions;
- an independent diagnostic detects forbidden drift;
- roll, pitch, airborne, and edge cases behave as required.

## Completion Report
When reporting a completed work unit, state:
- what changed;
- what verification was run;
- the important result;
- any remaining limitation.

If verification was not possible, explicitly report the work as unverified.