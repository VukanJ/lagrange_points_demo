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
The following options spawn many test bodies at a given lagrange point which are close together (the lagrange points are calculated
to the first order of the mass ratio M_planet / M_Sun and are therefore not exact.

Example:
```
./lagrange_points_demo L1
```
To create a ring of 10000 bodies on earths orbit use
```
./lagrange_points_demo R
```
Note: The blue circle shows the highest orbit of the planet (The orbit of the planet is slightly elliptical)

## Keyborard controls
* `R` key: Show reference frame in which the planet is visibly orbiting its sun
* `Arrow keys` Move camera
* `Mouse wheel` Zoom
* `D` Start deleting bodies which are too far away or too close to the sun
* `Mouse click` Spawns 2500 bodies at cursor
