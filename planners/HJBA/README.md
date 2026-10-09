# Hamilton–Jacobi-guided bidirectional A*

Xuemin Chi, Jun Zeng, Jihao Huang, Zhitao Liu and Hongye Su, *Fast Path Planning for Autonomous Vehicle Parking With Safety-Guarantee Using Hamilton–Jacobi Reachability*, IEEE Transactions on Vehicular Technology 75(8), 2026, pp. 15890–15904. [DOI](https://doi.org/10.1109/TVT.2026.3677282) · [Author manuscript](https://arxiv.org/abs/2310.15190).

`RunPlanner('HJBA',caseId)` returns a path. This independent MATLAB realization includes the numerical backward reachable tube, static safe-set intersection, twenty sampled connected states, asymmetric bidirectional search, and shortest-successful-branch selection. It does not replace the reachability layer with ordinary Hybrid A*. MATLAB Navigation and Parallel Computing Toolboxes are required; the planner itself does not use AMPL.

`BuildHJBA` optionally compiles the fused C++ implementation of the same HJ stencil with MATLAB's configured C++ compiler. On Windows, `BuildHJBA('path/to/zig.exe')` supports a portable Zig compiler. The binary and build caches live outside the repository, under `COMMONPARKING_HJBA_DIR` or the system temporary directory. `Plan` finds that cache automatically; without it, the complete MATLAB reference implementation runs. Published timings use the compiled stencil; its result is compared with the MATLAB stencil independently. The one-time build is excluded from planner time.

## Reachability and safety

The Dubins model has constant signed speed and bounded angular rate. For increasing look-back time, the minimizing Hamiltonian is `v*cos(theta)*Vx + v*sin(theta)*Vy - omegaMax*abs(Vtheta)`. We solve the HJ target variational inequality using Lax–Friedrichs dissipation, second-order ENO spatial derivatives, third-order SSP Runge–Kutta time integration, and projection onto the target sublevel function. This follows the level-set construction cited by the article: Fisac et al., *Reach-Avoid Problems with Time-Varying Dynamics, Targets and Constraints*, HSCC 2015, [author PDF](https://people.eecs.berkeley.edu/~sastry/pubs/Pdfs%20of%202015/FisacReach2015.pdf). That reference explicitly permits lower-order approximations; its example uses fifth-order WENO. XY boundaries use linear extrapolation and heading is periodic.

**Equation (2) prints a universal control quantifier, whereas its prose describes states from which the car can be driven into the target.** This implementation follows that controllable-target interpretation, using an existential control/minimizing Hamiltonian. It does not implement the different problem of reaching the target under every possible steering command.

Each full physical rectangle is tested against all convex obstacle polygons by separating axes. Strict separation is equivalent to a positive minimum convex-set distance, so this computes the same Boolean safe-set membership as the paper's pose-fixed QP without solving millions of QPs. No safety margin or enclosing-circle approximation is added. A separate test compares the Boolean result with independent primal quadratic programs.

The numerical tube is intersected with the safe grid states. Uniform sampling without replacement then enforces the literal equation (7): **global** `x >= xGoal` and `y >= yGoal`. We do not rotate this restriction into a goal frame or switch quadrants for individual layouts. The paper's angled-parking illustration appears to use points left of its goal; the printed restriction is retained here and can reduce useful samples in rotated scenes. The BRT uses a finite target neighborhood solely for guidance; every returned search path connects to the exact task goal.

Membership in this sampled intersection only establishes static safety at that state. It does not establish obstacle-free reachability of an entire connecting path. Numerical HJ approximation adds another limitation. Consequently all actual search and Reeds–Shepp edges receive continuous full-rectangle collision checks. Those analytic swept-arc/contact routines and the shortest RS conversion are shared with the `LaumondRS` folder; no Laumond planning stage is called.

## Online search

Each connected state gets a separate pair of searches, one rooted at the task start and the other at its goal. Both try a shortest Reeds–Shepp connection to that same connected state whenever a node is popped. The start-rooted search first considers forward steering primitives and permits only the two extreme reverse primitives if **all forward primitives are geometrically invalid**. A forward primitive already visited is not treated as a collision. The goal-rooted search considers both gear directions using its smaller grid and step. The prose in Section IV-B defines this rule more clearly than Algorithm 2's unconditional break statement; we follow that prose.

The accumulated cost is physical arc length. There are no reverse, gear-switch, or steering-change penalties. The XY diagonal/octile heuristic follows Section IV-C. Closed cells are not reopened; lower-cost open representatives are replaced. Each replacement creates an immutable node so parent chains retain the exact continuous pose that generated each edge. The goal-rooted half is physically reversed, including signed lengths, before concatenation at the exact connected pose. Among successful branches we choose the shortest total length. No later smoothing or fallback planner is inserted.

The article calls its diagonal heuristic consistent. For continuous primitives, diagonal distance can exceed Euclidean displacement; that argument does not establish admissibility here. We make no optimality or finite-grid completeness claim. RS joins preserve pose and bounded curvature, but **curvature can jump**, so no continuous-curvature claim is made.

## Published and chosen settings

| Setting | Value and provenance |
|---|---|
| Connected states | 20, paper |
| Forward/backward search XY cells | 0.5 / 0.3 m, paper Table II |
| Search heading cells | 5 degrees, paper Table II |
| Parallel branch workers | Up to 12, paper Table II; existing smaller pool is reported |
| HJ grid | 101 × 101 × 101; article uses this for angle/parallel and 61³ for perpendicular |
| Domain | Bounding box of task endpoints and all obstacle vertices, expanded by 5 m; chosen |
| Target half-widths | 0.35 m in X/Y and 8 degrees heading; chosen numerical guidance neighborhood |
| Dubins speed / angular limit | 1 m/s magnitude / `kappaMax` rad/s; chosen speed, common steering limit |
| Look-back horizon | 8 s per stage; chosen |
| PDE CFL factor | 0.4; chosen |
| Parallel cases 1–2 | Forward final-adjustment BRT, intersect with safety, then reverse-approach BRT; two-stage construction from Section V-B1, signs explicitly chosen |
| Other cases | One reverse-approach BRT; chosen fixed approach convention |
| Steering samples | Five equally spaced front-wheel angles; chosen |
| Primitive length | 1.5 times that search's XY cell width; chosen |
| Per connected-state branch budget | 15000 total popped nodes or 60 s; chosen |
| Random seed | 2026128 + case number; chosen |

The original offline/online distinction is retained in diagnostics, but this benchmark **recomputes the full grid, HJ tube and safe set on every call and includes all of them in planner wall time**. Pool startup/shutdown also count. These timings are not the article's online-only timings. Up to twelve process workers replace its parallel computation threads, capped by the local cluster's configured worker limit (eight on the release machine); each worker uses one numerical-library thread. A transient pool-start failure is retried once before any search starts, and both attempts count in wall time and are reported. A fixed seed plus wall-time stopping does not ensure identical results under different hardware load.

The output uses exact piecewise-circular geometry, uniform mileage separately within each gear run, and exact cusp samples. The common evaluator supplies the rest-to-rest timing and measures execution independently. Failed guidance or search remains a native failure.

`TestHJBA` checks a closed-form transport solution, angular reachability under mesh refinement, thirty independent SAT-versus-QP safety comparisons, and exact two-sided RS joins. Release verification additionally integrates every selected primitive with an independent ODE solver, checks all sampled guidance states, endpoint/cusp/mileage consistency, branch costs and native full-body collision freedom.
