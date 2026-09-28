# Algorithms

## A* global planning

For a grid node `n`, the priority is

```text
f(n) = g(n) + h(n)
```

where `g(n)` is accumulated path cost and `h(n)` is Euclidean distance to the goal. Cardinal moves cost `1`, diagonal moves cost `sqrt(2)`.

The implementation keeps a priority queue, best-known `g` values, and parent pointers for path reconstruction. Occupied cells are rejected before expansion.

### Complexity
For a grid represented as a graph, worst-case A* complexity depends on the heuristic and explored state space. With a binary heap, queue operations are logarithmic in the frontier size. In practice, an informative admissible heuristic reduces the number of expanded states relative to uninformed search.

## Path following

Given robot pose `(x, y, theta)` and a look-ahead waypoint `(x_g, y_g)`:

```text
d = sqrt((x_g-x)^2 + (y_g-y)^2)
theta_g = atan2(y_g-y, x_g-x)
e_theta = wrap(theta_g-theta)
v = clamp(k_v d)
omega = clamp(k_theta e_theta)
```

Forward velocity is attenuated when heading error is large. This simple controller is a baseline, not a claim of optimal trajectory tracking.

## Replanning behavior
The planner recomputes a path when new map/goal inputs are received. The known-map demo is deterministic so algorithm and controller behavior can be reproduced.
