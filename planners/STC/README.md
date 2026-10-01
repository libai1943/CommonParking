# STC

Reference: Bai Li, Tankut Acarman, Xiaoyan Peng, Youmin Zhang, Xuepeng Bian and Qi Kong, *Maneuver Planning for Automatic Parking with Safe Travel Corridors: A Numerical Optimal Control Approach*, European Control Conference, 2020, pp. 1993-1998. [Authors' source repository](https://github.com/libai1943/Automatic_Parking_Maneuver_Planning_ECC20).

This is a MATLAB/AMPL implementation based on the paper and the author's supplied MATLAB source (copyright Bai Li, GPL-3.0). The covering-disc extension and CommonParking integration are modifications; the results are not those originally reported in the paper.

## Algorithm and correspondence

Hybrid A* supplies a collision-checked path. An analytic, minimum-time one-dimensional speed profile is assigned separately to each gear run, with rest at each cusp. The resulting coarse trajectory is sampled uniformly in time. No CG smoother is used by this method.

Axis-aligned boxes are expanded around each disc centre in the paper's order: up, left, down, right. Each accepted box remains separated from every original obstacle by at least the covering-disc radius plus 0.01 m. Distances are computed against the convex obstacle polygons, avoiding raster inflation errors. The boxes stay fixed during each optimization.

The objective is terminal time only. The bicycle dynamics use the paper's **first-order explicit Runge-Kutta (forward Euler)** transcription. Position, heading, velocity and steering are states; acceleration and steering rate are controls. Start and goal pose and zero endpoint velocity, steering, acceleration and steering rate are imposed. Terminal heading uses the continuous branch reached by the initializer, equivalent modulo 2*pi for that branch. This avoids redundant sine/cosine equality constraints but does not search all possible winding numbers.

## Explicit settings and the multi-disc extension

The paper's Table I uses 100 sampled nodes, 0.03 m corridor expansion and a 10 m limit. The supplied MATLAB implementation uses 200 nodes, 0.1 m expansion and an 8 m limit; those supplied-code settings are used here, consistent with the requested common node count. Ipopt has 1,500 iterations / 90 CPU seconds per attempt; terminal time is bounded by 50 s. The front-end search uses 0.2 m / 0.2 rad cells, 0.7 m primitives, three steering choices, heuristic weight 3, and reverse/switch penalties 1/5. Search limits are raised to 100,000 expansions / 180 s. All values are visible in `+stc/Config.m`.

The original two circles circumscribe two half-body rectangles. As requested for these tighter cases, the vehicle rectangle can instead be partitioned in both length and width. Each cell is circumscribed by a circle. The union covers the **entire original vehicle**; no dimensions are reduced. Refining both directions reduces the excess coverage around all four sides, unlike merely adding circles on the centreline.

The deterministic partitions are 2x1, 4x2, 6x3, 8x4, 10x5, 12x6, 16x8 and 20x10, giving 2, 8, 18, 32, 50, 72, 128 and 200 discs. A partition is skipped when its discs overlap obstacles at seed nodes. If a fixed-corridor NLP fails, the next partition is tried from the same initial trajectory. The first native successful solve is returned. Attempts and solver flags are retained. The source's relocation of obstructed box seeds is not used here: a more accurate covering partition is tried instead.

These changes retain STC's fixed-corridor optimization mechanism. They are a documented extension, not an assertion of an exact two-disc reproduction. Collision avoidance is imposed at the 200 planner nodes; the independent common evaluator measures what happens during dense execution between nodes.

## Usage and outcomes

```matlab
result = RunPlanner('STC', 6);
caseData = LoadCase(6);
evaluation = EvaluateResult(result, caseData);
PlotEvaluation(result, evaluation, caseData);
```

`result.kind` is `trajectory`. Only a native AMPL/Ipopt solved flag and valid numeric output count as planner success. A failed attempt is never relabelled as success merely because it wrote a candidate. All attempts, including unsuccessful coarser covers, are included in planner wall-clock time. Runtime files remain outside the repository.

The released twelve-case run solved all cases. All tracked results have zero measured collision frames and satisfy the common terminal tolerance. Case 6 required 128 covering discs; the others used 2-50. This is an empirical result for the frozen suite and published limits, not a general completeness guarantee. See the root results table and `results/STC/validation.json`.
