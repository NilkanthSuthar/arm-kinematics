# arm-kinematics

A small C++17 library for the kinematics of a **2-link planar robot arm**.

```
            (x, y)  tool tip
             o
            /
       l2  /  theta2 (relative to link 1)
          /
   elbow o
        /
   l1  /  theta1 (from the +x axis)
      /
     o---------->  x
   base (0, 0)
```

## What it does

`arm::PlanarArm` (in [`src/arm.hpp`](src/arm.hpp)) gives you:

| Function | What it does |
| --- | --- |
| `PlanarArm(l1, l2, shoulder_limit, elbow_limit)` | Creates an arm. Limits are optional and default to `[-pi, pi]`. |
| `forward(theta1, theta2)` | Joint angles → tool tip `(x, y)`. Throws `std::out_of_range` if an angle breaks a limit. |
| `inverse(x, y, elbow_up)` | Tool tip → joint angles. Returns `std::nullopt` if the point is unreachable or the solution breaks a limit. |
| `linear_trajectory(start, end, steps, elbow_up)` | Straight-line tool path → joint angles at `steps + 1` evenly spaced waypoints. Returns `std::nullopt` (never a partial path) if any waypoint fails. |
| `set_joint_limits(shoulder, elbow)` / `within_limits(theta1, theta2)` | Change or check the joint limits. |

Angles are in radians, counter-clockwise positive. Lengths can be in any unit, as long as you use the same one everywhere.

```cpp
#include "arm.hpp"

arm::PlanarArm robot(2.0, 1.0);

arm::Point tip = robot.forward(0.3, 0.5);                // where is the tool?
auto angles = robot.inverse(1.5, 1.2, /*elbow_up=*/true);  // how do I get there?
if (angles) {
    // use angles->theta1, angles->theta2
}

auto path = robot.linear_trajectory({2.5, 0.5}, {1.0, 2.0}, 20);
if (!path) {
    // some point on the line is out of reach
}
```

## The math in 5 lines

1. Forward: `x = l1·cos(θ1) + l2·cos(θ1 + θ2)`, `y = l1·sin(θ1) + l2·sin(θ1 + θ2)`.
2. Law of cosines for the elbow: `cos(θ2) = (x² + y² − l1² − l2²) / (2·l1·l2)`.
3. The target is reachable only if that value is in `[-1, 1]`, which means `|l1 − l2| ≤ √(x² + y²) ≤ l1 + l2`.
4. `θ2 = ±acos(...)`. Negative is **elbow up**, positive is **elbow down**. At full extension (`θ2 = 0`) the two are the same, which is the singular case.
5. Shoulder: `θ1 = atan2(y, x) − atan2(l2·sin(θ2), l1 + l2·cos(θ2))`.

## Build and test

You need CMake 3.16 or newer, a C++17 compiler and an internet connection the first time, because GoogleTest is downloaded with `FetchContent`.

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

### Coverage (GCC + lcov)

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DARM_ENABLE_COVERAGE=ON
cmake --build build
ctest --test-dir build
lcov --capture --directory build --include "$PWD/src/*" --output-file coverage.info --ignore-errors mismatch
lcov --summary coverage.info
```

### Formatting, linting and docs

```bash
clang-format --dry-run --Werror src/*.hpp src/*.cpp tests/*.cpp
clang-tidy -p build src/arm.cpp tests/test_arm.cpp
doxygen
```

`.clang-tidy` turns on the `modernize-*`, `bugprone-*` and `readability-*` checks and treats every warning as an error. The few checks that are switched off are listed, with the reason for each, in [`.clang-tidy`](.clang-tidy) and [`tests/.clang-tidy`](tests/.clang-tidy). `doxygen` writes HTML to `docs/html/` and fails if a public function has no doc comment.

CI ([`.github/workflows/ci.yml`](.github/workflows/ci.yml)) runs all of the above on every push and pull request.

## Coverage

**100% line coverage** (58 of 58 lines) and **100% function coverage** (10 of 10 functions) of `src/`, measured with gcov/lcov from GCC 13 on Ubuntu 24.04. CI prints the current number on every run.

## Limitations

- **2D only.** The arm moves in a plane. There is no base rotation, no wrist and no third dimension.
- **No dynamics.** Only positions are computed. There are no velocities, accelerations, torques, masses or timing. `linear_trajectory` gives waypoints, not a time-parameterised motion.
- **No collision checking.** The arm can pass through itself, the floor or any obstacle. Only joint limits and reach are checked.
- **Trajectories are checked at waypoints only.** The path between two waypoints is not checked. Use enough `steps` for your workspace.
- **Angle wrap-around.** `theta1` is returned in `[-pi, pi]`, so a path that crosses the −x axis can jump from `+pi` to `-pi` between waypoints, even though the arm hardly moves.
