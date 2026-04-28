# vehicle-bridge

A C++ library for vehicle communication and control, part of the OpenBuilder demo project.

Provides:
- `VehicleState` — position, velocity, heading, and steering angle representation
- `CanMessage` — simplified CAN bus message parsing
- `VehicleController` — send throttle, brake, and steering commands

## Dependencies

- [common-utils](https://github.com/wenjunli312-code/common-utils)

## Build

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
cmake --install build
```
