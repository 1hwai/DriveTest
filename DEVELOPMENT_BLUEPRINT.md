# DriveTest Development Blueprint

> **Purpose:** Project-wide roadmap and shared source of truth for planning DriveTest through its first public release, **v0.1.0**.
>
> **Status:** Living document. Update it when priorities, architecture, scope, or milestone completion changes. A planned feature is not complete until it exists in the repository and has been tested.
>
> **Scope:** v0.1.0 is the first usable, reproducible release, not the final vision for a full rally simulator.

## 1. Project vision

DriveTest is a long-term C++ rally-driving engine and game project. Its first release should provide a coherent, playable driving experience and a foundation that can be extended without repeatedly rewriting unrelated systems.

The long-term direction includes:
- Reusable vehicle simulation: physics, drivetrain, suspension, tires, input, and audio.
- Long-form rally stages with varied roads, elevation, bumps, terrain, and roadside scenery.
- An in-engine level editor as the default stage-authoring tool.
- External asset and stage import, especially from Blender, through a documented import pipeline.
- Mod-friendly content organization for vehicles, stages, scenery, and materials.
- Clear separation between engine, editor, content data, importers, and runtime.

The first release must remain deliberately smaller than this full vision. Extensibility is a design goal, but speculative systems should not delay a usable game.

## 2. Product principles

### 2.1 Separate content from code
Stage layouts, vehicle tuning, materials, scenery placement, and other content should live in data files wherever practical, not as hardcoded test cases in engine source.

### 2.2 Keep responsibilities explicit
Rendering, physics, vehicle simulation, world/stage data, input, audio, importing, and editing should have clear boundaries. Importers should not directly manipulate the physics world; vehicle tuning should not require changes to rendering code.

### 2.3 One internal representation, multiple authoring tools
The built-in editor and external tools should ultimately produce the same internal stage and asset data. Blender is an authoring option, not a runtime dependency.

### 2.4 Prefer incremental, testable development
Implement small end-to-end capabilities and verify them before adding more layers. Avoid building large editor frameworks, generic plugin systems, or advanced simulation features before a concrete need exists.

### 2.5 Keep the project buildable
After meaningful changes, build on available development platforms and run relevant smoke tests. Record failures honestly; an untested change is not verified.

### 2.6 Distinguish goals from facts
This roadmap describes intended work. The source code, tests, and release notes determine what is actually complete.

## 3. Architecture direction

The following is a conceptual responsibility map, not a requirement to create these exact directories immediately.

- **Core/Application:** application lifecycle, timing, logging, configuration, and subsystem coordination.
- **Rendering:** graphics resources, shaders, materials, camera, meshes, and draw submission.
- **Physics:** rigid-body integration, collision detection, raycasts, contact generation, solver, and road/terrain collision.
- **Vehicle:** chassis, wheels, suspension, tires, powertrain, transmission, differential/torque distribution, brakes, steering, and configuration.
- **Input:** keyboard and controller abstraction, action mapping, and edge-triggered actions.
- **Audio:** engine, exhaust, tire/road interaction, and other driving feedback.
- **World/Stage:** stage lifecycle, terrain, road geometry, scenery instances, checkpoints, and spawn points.
- **Assets/Importers:** file decoding, validation, conversion into engine-owned data, and error reporting.
- **Editor:** stage-authoring UI operating on stage data without owning runtime physics or rendering internals.
- **Content:** vehicle definitions, stages, materials, textures, meshes, and configuration.
- **Tools/Tests:** diagnostics, repeatable test scenarios, and validation.

Do not split files or create modules solely to match this list. Refactor when a real responsibility boundary or workflow benefit justifies it.

## 4. Milestones toward v0.1.0

Milestones are ordered by dependency, but work may overlap when safe. Completion requires implementation and verification.

### M0 — Reliable development baseline
**Goal:** Developers can identify the source of truth and build a known revision.

- Keep CMake and dependency setup reproducible on supported development platforms.
- Keep generated build output, logs, and machine-specific files out of source control.
- Document build, run, content, and troubleshooting steps.
- Maintain a smoke-test checklist for startup, rendering, world loading, physics stepping, and shutdown.

**Exit criteria:** A clean checkout can be built using documented steps; known limitations are recorded.

### M1 — Engine and world foundation
**Goal:** A stable application and world lifecycle with clear ownership.

- Maintain explicit ownership of renderer, mesh/asset resources, world, camera, and runtime objects.
- Keep initialization and shutdown order predictable.
- Avoid hardcoded demonstration objects or track details in core runtime code.
- Make stage/world creation and loading data-driven.
- Preserve a boundary between rendering and physics representations.

**Exit criteria:** The application starts, loads a stage, runs, and shuts down consistently without accidental initialization-order dependencies.

### M2 — Vehicle simulation baseline
**Goal:** A car that can be driven repeatedly and tuned systematically.

- Rigid-body integration, collision handling, contacts, and solver stability.
- Suspension travel, spring/damper forces, wheel raycasts, and chassis-ground clearance.
- Tire forces, steering geometry, braking, and traction.
- Engine torque curve, RPM, clutch, gears, transmission, and differential/torque distribution.
- Vehicle parameters stored in configuration files.
- Fixed physics timestep and useful diagnostic telemetry.

**Exit criteria:** The car can accelerate, brake, steer, shift, traverse uneven terrain, and recover from ordinary bumps without solver explosions or persistent contact instability. Document known physical approximations.

**Not required for v0.1.0:** Motorsport-grade tire modeling, a universal vehicle dynamics framework, or exact simulation of every drivetrain configuration.

### M3 — Driving controls and feedback
**Goal:** Driving is understandable and players can tell what the car is doing.

- Keyboard controls and the existing gamepad path.
- Configurable actions where practical; reliable press-edge handling for one-shot actions such as shifting.
- Steering, throttle, brake, clutch, and gear controls.
- Player-facing speed and gear/RPM feedback, distinct from debug-only inspector values.
- Useful physics/vehicle telemetry for development.
- Basic audio feedback where feasible: engine RPM, exhaust character, and tire/road interaction.

**Exit criteria:** A player can drive without keeping the debug inspector open. Unsupported controller features must fail gracefully.

### M4 — Rally stage and terrain
**Goal:** A stage large and varied enough to demonstrate rally driving rather than a tiny physics test area.

- Long road paths with configurable width, curves, elevation, bumps, and banking where appropriate.
- Terrain beyond the road, including hills, dips, and roadside transitions.
- Road and terrain collision that reasonably match visible surfaces.
- Reusable vegetation and roadside scenery placement.
- Spawn points and stage settings stored as content data.
- Configurable stage dimensions, not a tiny hardcoded test area.

**Exit criteria:** At least one coherent test stage can be loaded repeatedly and driven from start to finish, with important stage parameters editable without source-code changes.

### M5 — Content loading and asset pipeline
**Goal:** Content can be added without embedding asset-specific logic in engine systems.

- Establish internal mesh/material/texture representations.
- Add a GLB/glTF import path when the renderer and asset architecture are ready.
- Convert external assets into engine-owned data through a dedicated importer layer.
- Validate missing files, unsupported features, invalid transforms, and malformed metadata with actionable errors.
- Define units, coordinate conventions, orientation, naming, material handling, and collision metadata.
- Keep source assets distinguishable from runtime-ready assets; add caching/preprocessing when useful.

**Exit criteria:** A simple external model imports, displays with expected transforms/materials, and requires no special-case renderer code.

### M6 — Stage format and built-in level editor
**Goal:** The built-in editor becomes the normal tool for creating rally stages.

Build incrementally:
1. Load, save, and switch between multiple stages.
2. Select, move, rotate, and scale scene objects.
3. Place and duplicate props such as trees, rocks, signs, and barriers.
4. Edit road paths, road width, curves, elevation, and surface settings.
5. Sculpt terrain with raise/lower/smooth brushes and refine road edges.
6. Configure spawn points, checkpoints, environment, and stage metadata.
7. Preview the stage in the runtime and identify invalid or missing data.

The editor should manipulate a stage-data model. The runtime should consume saved stage data through the same loading path used by packaged stages. Avoid incompatible editor-only and runtime-only stage representations.

**Exit criteria:** A user can create or modify a basic stage, save it, restart the application, and load the same stage without editing engine source.

### M7 — External stage authoring and mod support
**Goal:** External tools and community content coexist with the built-in editor.

- Document and version a stage interchange format.
- Import Blender-authored meshes and supported scene metadata.
- Use explicit conventions or metadata for road paths, terrain, collision meshes, spawn points, checkpoints, and prop instances.
- Do not assume a rendered road mesh alone contains enough information to reconstruct road width, centerline, banking, or gameplay semantics.
- Imported stages should become ordinary internal stage data and remain editable in the built-in editor where possible.
- Define content package structures for vehicles, stages, and shared assets.
- Validate packages and report incompatible format versions or missing dependencies.

**Exit criteria:** At least one externally authored asset imports through a documented pipeline. Full Blender-to-stage conversion and community distribution may remain post-v0.1.0 unless the core editor and format are already stable.

### M8 — Packaging, polish, and v0.1.0 release
**Goal:** Deliver a small but coherent, reproducible first release.

- Select and document supported operating system(s) for the first release.
- Provide a clean build/launch procedure, required dependencies, and sample content.
- Include one polished playable rally stage and a configured vehicle.
- Ensure basic controls, driving feedback, stage loading, and shutdown are reliable.
- Fix release-blocking crashes, severe physics failures, broken content paths, and obvious regressions.
- Include known limitations, controls, build instructions, and a concise changelog.
- Verify from a clean checkout/build directory and test the packaged result, not only the development executable.

**Exit criteria:** A new user can follow the documented procedure, launch the project, load the included stage, drive the car, and understand known limitations.

## 5. Stage and asset data design

The exact format is not yet fixed. Choose it based on actual needs rather than committing prematurely to a large schema.

A stage will eventually need to represent:
- Format/schema version and stage identity.
- Terrain data and surface/material assignments.
- Road paths, widths, elevation, banking, and surface type.
- Placed objects, transforms, asset references, and optional instance settings.
- Collision configuration where it cannot be safely derived from visible meshes.
- Player spawn, checkpoints, environment settings, and stage metadata.

A vehicle package may contain:
- Model and material assets.
- Wheel locations, axes, and visual wheel references.
- Collision representation.
- Vehicle configuration such as mass properties, suspension, tires, brakes, engine, gearing, and torque distribution.
- Optional audio references and a package manifest.

Prefer readable text metadata during early iteration. Use binary or specialized formats for large data such as terrain heightfields only when justified. Version formats and define compatibility behavior before relying on them for public mods.

## 6. Level editor and external tools

The built-in level editor is the primary authoring workflow. Blender is complementary for modeling and selected external scene-authoring tasks.

Long rally stages should not require hand-modeling every meter of road or placing every tree individually. Prioritize:
- **Road-first workflows:** draw/edit a centerline and control width, elevation, curvature, and banking.
- **Terrain generation:** generate terrain around the road, then refine it with local sculpting.
- **Scattering and instancing:** distribute vegetation and roadside objects efficiently.
- **Reusable assets and presets.**
- **Stage preview and validation.**

Later, the editor may be distributed as a separate application so creators can author stages without running the full game. Keep stage editing independent from live runtime state so this remains possible. Do not create a separate editor executable until there is a practical benefit and the data/loading boundary is stable.

## 7. Testing and diagnostics

Testing should grow alongside systems rather than being postponed until release.

- **Build smoke test:** clean configure/build and launch.
- **Physics tests:** rigid-body integration, static-body handling, raycasts, collision contacts, and solver stability.
- **Vehicle tests:** suspension travel, wheel-ground contact, braking, acceleration, gear changes, and drivetrain behavior.
- **Content tests:** load known assets/stages, validate references, and report malformed files.
- **Editor tests:** save/load round trips and persistence of transforms and stage settings.
- **Runtime smoke test:** start stage, drive, pause/debug if supported, reload or exit safely.

Use logs and debug visualization to investigate issues. Avoid tuning physics based only on a screenshot or impression; use repeatable scenarios and telemetry when possible.

## 8. Performance and scope management

Prioritize correctness and a measurable playable loop before optimization. Track frame rate and physics update rate, but do not add complex streaming, chunk loading, multithreading, or sophisticated LOD systems until real stage size and profiling justify them.

For the first release:
- One vehicle is acceptable.
- One well-made stage is acceptable.
- Basic scenery is acceptable.
- Limited audio is acceptable.
- A simple editor is acceptable.
- Known simulation approximations are acceptable if they do not undermine basic playability and are documented.

Do not block v0.1.0 on a large vehicle/stage collection, online multiplayer, AI opponents, replays, career progression, an economy, a complete standalone editor distribution, a mod marketplace, or a universal plugin framework.

## 9. Working workflow

For each significant change:
1. State the problem and desired behavior.
2. Identify the responsible subsystem and relevant files.
3. Prefer a focused change that preserves established boundaries.
4. Build and run relevant checks where possible.
5. Record the outcome, limitations, and follow-up work.
6. Update this blueprint only when a milestone, priority, architecture decision, or completion status materially changes.

Keep changes reviewable. Avoid unrelated formatting rewrites and broad refactors mixed into physics or feature work. Understand file responsibilities before editing multiple files.

## 10. Known status at the time this blueprint was written

This is a planning snapshot, not a guarantee that every item is still current. Re-check the repository and runtime before relying on it.

- C++17 project using CMake, SDL3, OpenGL, and glad.
- Core application, renderer, world, object, mesh, and debug UI infrastructure exist.
- Physics includes rigid-body, collision, contact/solver, and raycast systems.
- Vehicle code includes wheel/suspension and powertrain-related systems with external vehicle configuration. `VehicleConfigWatcher` now owns file timestamp polling and transactional reload parsing; `Scene` applies accepted values to the live car. The current scene still hardcodes the TestCar config and Subaru model, so a selectable multi-vehicle catalog remains future work.
- A test stage/road and roadside scenery systems exist.
- Gamepad input has been introduced.
- Audio initialization previously failed in at least one environment due to unavailable audio devices; verify the current environment before treating audio as functional.
- Suspension configuration, AWD torque distribution, road path shape, terrain bumps, and wheel visual spin direction have been adjusted in recent work, but require local runtime verification.
- A versioned text stage format now stores terrain generation settings, elevation profiles, bumps, and road control points. `StageSerializer` supports validated load/save, and the test track is generated from `Stages/TestTrack.stage` rather than hardcoded terrain and road arrays. Roadside scenery placement is still procedural, and the in-engine editor UI, terrain sculpting, object placement, and stage switching remain future work.
- A tire telemetry panel displays per-wheel normal load and combined longitudinal/lateral slip velocity from the tire model. Visual/runtime behavior still requires local verification.
- A complete GLB importer and the intended in-engine terrain/road editor remain future work unless current source code shows otherwise.

## 11. Release checklist

Before tagging v0.1.0, confirm all applicable items:

- [ ] Clean checkout builds with documented dependencies.
- [ ] The application starts and exits reliably.
- [ ] A vehicle can be controlled with documented inputs.
- [ ] Acceleration, braking, steering, shifting, and suspension are usable.
- [ ] The included stage is varied enough to demonstrate rally driving.
- [ ] Visible road/terrain and collision behavior are acceptably aligned.
- [ ] Stage and vehicle settings are not needlessly hardcoded in engine source.
- [ ] Content paths work from a clean build/package.
- [ ] Basic player-facing driving feedback is available.
- [ ] Critical logs and known limitations are documented.
- [ ] Relevant physics/content tests and runtime smoke tests have been performed.
- [ ] The packaged release has been tested separately from the developer environment.
- [ ] Release notes describe included features, omissions, and known issues.

## 12. Maintaining this blueprint

This is the project-level roadmap, not a substitute for detailed technical design documents or issue tracking.

- Keep milestone order and release criteria here.
- Put detailed design decisions in focused documents under docs/ when needed.
- Use issues or task lists for individual bugs and implementation tasks.
- Mark work complete only after verification.
- When priorities change, update this file and record why.
- If implementation diverges from the plan, either correct the implementation or revise the plan explicitly; do not let the document silently become fiction.
