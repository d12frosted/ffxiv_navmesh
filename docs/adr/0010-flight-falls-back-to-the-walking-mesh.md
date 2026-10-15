# 0010. Flight falls back to the walking mesh

## Status

Accepted.

## Context

A flying route out of Bestways Burrow in Mare Lamentorum to a FATE on the surface makes the
character fly into the rock. The route a player takes is through the central section of the
burrow and up the tunnel to the surface. The plugin instead heads for the far wall of the
chamber under the destination and pushes against it. The reverse route fails the same way.

Reproduced off the game thread on the cached zone (`u5f1`), from the exact start and goal the
log recorded, with the volume search in each of its configurations:

| search | expansions | time | result |
|---|---|---|---|
| tuned as configured (weight 1.5, raycast off, budget 100000) | 100000 | 275 ms | stops 738 y short |
| tuned as ADR 0008 defaults (raycast on, budget 200000) | 200000 | 781 ms | stops 732 y short |
| upstream (weight 1, raycast on, budget 1000000) | 1000000 | 11.5 s | stops 732 y short |
| weight 1.5, raycast off, no budget | 854591 | 0.9 s | reaches the goal, 1826 y |
| weight 1, raycast off, no budget | 1874861 | 2.6 s | reaches the goal, 1594 y |

So the volume does connect the two ends, and the search does find the route given enough
budget: the shortest voxel route is 1605 y against a straight line of 1036 y, and it goes out
through the central section exactly as a player would. The problem is the shape of the
search. A* has to expand every node whose estimate falls below the length of the real route,
and the real route spends its first 700 y underground, moving away from the goal. Underground
is all leaf voxels, so that is a million expansions before the tunnel is even considered. No
budget the plugin can afford covers it, and upstream's own budget does not either.

When the budget runs out, `VoxelPathfind` hands back the path to the node closest to the goal
and `PathfindVolume` appends the goal itself as the final waypoint. That last hop is the one
through the wall.

Two things were tried and do not work.

**A coarser heuristic.** Running a distance field on the 16 y cells of the octree, so the
search knows where the exit is, fails because of what the volume actually stores. The
voxeliser rasterises surfaces, so the inside of rock is empty as far as the volume is
concerned, and every 16 y cell contains empty voxels. A coarse grid over it says the burrow is
1165 y from the FATE, barely more than the straight line, because it walks straight through
the moon. The volume does not know where the walls are at any resolution but the finest.

**Connected component labelling** (`docs/backlog.md` item 3) does not apply. The ends are in
the same component. This is a different failure from the one that item describes: reachable,
but not within budget.

What does know the shape of the zone is the walking mesh. Detour finds the same route in tens
of milliseconds, because a mesh is a compact description of the surfaces, and the corridors a
walker needs are the corridors a flier needs when the air between two points is blocked.

## Decision

**When the volume search stops short of the goal, ask the walking mesh for the route and follow
it through the volume leg by leg.** `MeshCorridor.Corners` runs Detour's search between the two
points and returns the corners of the string-pulled result. `VoxelCorridor.Follow` lifts each
corner into the nearest empty voxel, since a corner sits on a floor and the floor is a solid
voxel, then joins consecutive corners: in line of sight, the corner is taken as it is; otherwise
a volume search runs between the two, which is short because the corners were chosen to be
mutually visible along the floor. The result goes through the same straightening as any other
volume path.

It runs only when the primary search returned a path whose last voxel is not the goal, so a
query the search can answer costs exactly what it did before. It is behind
`VolumeMeshFallback`, off by default as ADR 0002 requires.

Three rules keep it from being expensive when it cannot help either:

- **Any leg that cannot be finished abandons the corridor**, and the original partial path is
  returned. A chain with a gap in it is exactly what this replaces.
- **A leg may spend at most 20000 expansions.** A leg that needs more is a leg with a problem.
  The caller's budget still applies when it is smaller.
- **The mesh query is sliced and capped at 20000 iterations.** A walking route Detour cannot
  complete is otherwise a flood of the whole mesh, which on a large zone is a few hundred
  milliseconds spent after the volume search already failed. The capped query finalises to its
  best partial route.

**The two ends are the nearest mesh points the volume agrees can be flown to.** The nearest
polygon by distance alone is the wrong one surprisingly often: a flier over a slope has its
nearest polygon 7 y sideways, outside the walking query's box, and a flier over a crater wall
has the floor of a cavern 336 y straight down through rock as its nearest. So the lookup takes
a wide and tall box (150 y sideways, 400 y up and down), sorts what it finds by distance, and
tries them nearest first with a short volume search of 2000 expansions, at most 32 tries
spread over distinct 16 y cells so one unreachable patch of mesh cannot use them all up.

A partial walking route is used when it is the best on offer. Detour stops at the reachable
polygon nearest the goal, and where that is matters: from the burrow the walking route ends
196 y from the FATE at the edge of the surface, one easy flight away, while from the FATE it
ends 806 y away with the whole problem still ahead. So when the route is partial the reverse
route is asked for too, and the one that leaves the shorter gap is used, reversed if need be.

Measured on the route above, both directions and both tuning profiles:

| route | profile | primary search | fallback | result |
|---|---|---|---|---|
| burrow to FATE | as configured | 80 ms, 736 y short | 109 ms | 1927 y, straightened to 92 points |
| FATE to burrow | as configured | 82 ms, 377 y short | 105 ms | 1937 y, 66 points |
| burrow to FATE | ADR 0008 defaults | 691 ms, 732 y short | 99 ms | 1929 y, 87 points |
| FATE to burrow | ADR 0008 defaults | 685 ms, 326 y short | 102 ms | 1934 y, 66 points |

Measured on the other routes the same evening's log recorded, as configured:

| route | primary search | fallback | result |
|---|---|---|---|
| surface to the eastern underground FATE, from 3 y above mesh | 746 y short | 229 ms | 2160 y |
| the same, from 15 y above a slope | 506 y short | 269 ms | 1629 y |
| the same, hovering over the crater wall | 506 y short | 43 ms | declined |
| eastern underground FATE to the surface FATE | 1130 y short | 129 ms | 2368 y |

The declined one is a start with no walkable mesh within 140 y, over terrain so rugged that
the air above it is 170000 leaf voxels; reaching any mesh from there is a volume search of its
own, and the endpoint lookup gives up at 43 ms rather than run it. The character was only
there because it had been following a partial path into the wall, which with the fallback on
would not have happened. It is a real limit of the approach all the same.

The corridor route is about a third longer than the optimum the unbounded search finds (1456 y
straightened), because it hugs the walking route rather than cutting across the chambers. For a
route that was otherwise a flight into rock that is accepted.

Measured on 60 random walkable pairs 150 to 500 y apart per zone, ADR 0008 defaults:

| zone | primary failed | corridor rescued | fallback cost per failure |
|---|---|---|---|
| Mare Lamentorum (`u5f1`) | 12 of 60 | 0 | 88 ms |
| Yak T'el (`x6f2`) | 18 of 60 | 1 | 91 ms |

Random pairs are rarely rescued, and that is expected: those failures are mostly the sealed
pockets of backlog item 3, a hilltop against the inside of a building, which no corridor
connects either. The fallback is for the corridor class of failure, which random sampling
almost never produces and play produces whenever a route starts or ends indoors. What the
sweep establishes is the cost: about 100 ms on a query that already spent its whole budget,
against roughly 600 ms before the two caps.

## Consequences

- A fresh install is unchanged. With the switch on, a flying route the search cannot finish
  follows the walking route out instead of the wall, when the mesh knows one.
- The fallback flight stays low, along the floor of the walking route. Through a tunnel that is
  the only route there is; across open ground the primary search normally succeeds and this
  never runs.
- A hopeless query costs about 100 ms more than before. It already cost the full search budget
  and returned a path into a wall, so this is the cheaper part of the failure.
- Walking routes that go through off-mesh links (cosmoliners, drops, the shortcuts in the
  customisations) produce legs the volume may not be able to follow. The corridor then
  declines, which is no worse than before. Following such a link in flight is not attempted.
- A partial walking route whose gap the volume cannot bridge still fails, and backlog item 3
  stays open for the sealed pocket case. A start far from any mesh over rugged ground fails too,
  by design of the endpoint lookup; see the crater wall route above.
- One line is logged at Info level whenever the fallback runs: how far the search fell short,
  whether the mesh had a route, and whether the corridor was followed. Diagnosing this problem
  needed exactly that and the log did not have it.
- `tests/Navmesh.Tests/VoxelCorridorTests.cs` builds a cave whose only exit is away from the
  goal, shows the plain search fall short in it, and checks that the corridor reaches the goal
  without any hop passing through geometry, that it declines cleanly, and that a hopeless leg
  stops at the leg budget. The mesh side needs a Detour mesh and is not covered.
- The merge checklist from ADR 0002 grows by one hunk in `NavmeshQuery.cs`, marked with this
  ADR's number, and one checkbox in the recorder tab. `NavVolume/VoxelCorridor.cs` and
  `Movement/Human/MeshCorridor.cs` are new files and do not conflict.
