# lagrange_points_demo
Very simplistic simulation of restricted 3-body problem (Sun + Planet + N mass probes)
to demonstrate the stability of lagrange points using Euler integration.

## Prerequisites
* SFML
* Eigen3

## Installation
```
mkdir build && cd build
cmake ..
make
```

## Predefined setups
The following options spawn test bodies at a given lagrange point (the lagrange points are calculated
to the first order of the mass ratio M_planet / M_Sun and are therefore not exact.

Example:
```
./lagrange_points_demo --L1
```

## Keyborard controls
* `R` key: Show reference frame in which the planet is visibly orbiting its sun
* `Arrow keys` Move camera
* `Mouse wheel` Zoom
* `D` Start deleting bodies which are too far away or too close to the sunints_demo
