# Vehicle assets and configuration

Each vehicle should own a directory under `Assets/Vehicles/`:

```text
Assets/Vehicles/
  TestCar/
    vehicle.ini
  AnotherCar/
    vehicle.ini
```

`vehicle.ini` contains the tuning parameters for that vehicle: mass and collider dimensions, wheel positions, suspension, tire behavior, steering/brakes, engine, and transmission. Keep vehicle-specific tuning out of C++ defaults whenever a value is intended to be adjusted during development or by modders.

## Runtime tuning

The active test vehicle's `vehicle.ini` is checked every 0.5 seconds while the simulation runs. Save a valid edit to apply it without restarting. If parsing or validation fails, the previous live configuration stays active; fix the file and save again.

Most simulation parameters apply to the existing vehicle. `spawnClearance` is used only when the vehicle is first spawned, so changing it does not reposition a car that is already driving. Collider size and mass are updated on reload.

## Current limitation

The current scene still selects `TestCar/vehicle.ini` and loads the Subaru model through a fixed path in `Scene.cpp`. Separate per-vehicle config folders establish the data organization, but choosing among multiple complete vehicle definitions (config + model + wheel assets) is a later vehicle/content-loading task. Do not assume that adding another folder alone makes a second selectable car appear in-game.
