# CC-Steer with a directed probabilistic-roadmap adapter

The steering method is Thierry Fraichard and Alexis Scheuer, “From Reeds and Shepp's to Continuous-Curvature Paths,” IEEE Transactions on Robotics 20(6), 1025–1035, 2004, [DOI](https://doi.org/10.1109/TRO.2004.833789), [authors' final manuscript](https://homepages.loria.fr/AScheuer/Old_W3/Articles/Fraichard_Scheuer_IEEETRA_04.pdf).

**CC-Steer is an obstacle-free local connector, not a complete obstacle planner by itself.** This folder reproduces that connector and embeds it in an explicitly chosen directed PRM. It does not claim to reproduce the paper's PPP/ACA experimental planners, their timings, or a globally shortest parking path. In particular, this finite nearest-neighbor roadmap is not a completeness proof.

## Paper construction and source provenance

States at graph nodes are `(x,y,theta,kappa=0)`. A CC turn comprises clothoids and possibly a constant-curvature arc; straights and turns join with continuous curvature, including at direction changes. Curvature derivative may jump. The connector uses the nine Section IV-C/Eq. (17) families: `TTT`, `TcTT`, `TTcT`, `TTcTT`, `TcTTcT`, `TcTSTcT`, `TcTST`, `TSTcT`, and `TST`, where `c` denotes a cusp. Empty, straight, single-turn and two-turn degeneracies are retained. Irregular turns are permitted, as in the paper.

The circle/tangent and Fresnel implementation comes from Holger Banzhaf's later [steering_functions](https://github.com/hbanzhaf/steering_functions/tree/79b9217697b681d953768ea0d95a9307bd312a0e) CC00 implementation, under its retained Apache-2.0/BSD notices. Its additional `TcTcT`, `TcST`, `TScT`, and `TcScT` family selection is disabled to match the 2004 printed family list. This is licensed reuse, not a claim that this C++ code was supplied by Fraichard and Scheuer.

That source alone omits the essential Section IV-D **topological path**. `native/cc_topological.hpp` implements it separately: a backward elementary turn matches heading, a straight reaches the perpendicular line through the goal, and two forward elementary turns separated by a backward straight remove the lateral displacement. The shortest candidate among the family construction and this topological path is returned. Collision rejection does not silently substitute another family.

For heading change `delta` and direction `d`, an elementary pair has half-length `h=max(sqrt(abs(delta)/sigma_max),abs(delta)/kappa_max)` and sharpness `delta/(d*h^2)`. This is the minimum feasible symmetric-clothoid construction of Appendix A/Eqs. (19)–(23). For lateral displacement, 65 bisections solve `2*r(alpha)*sin(alpha)/cos(2*alpha)=abs(displacement)`, with `0<alpha<pi/4` and `r` the corresponding elementary-turn chord. This implements Appendix B/Eq. (24). **Appendix B says the perpendicular line passes through the start; Section IV-D2 says the goal. The implementation follows IV-D2**, which makes the lateral construction terminate at the goal. Numerical zero guards are 1e-14 rad for turns, 1e-13 m for straight pieces, and 1e-12 m for lateral displacement.

## Disclosed obstacle-planner adapter

All settings are in `+ccp/Config.m`, uniform across the 12 cases:

| Setting | Value |
|---|---|
| Graph budget | 2,500 nodes, including the two task endpoints, or 60 s |
| Random sampling | Uniform XY over scene/endpoints bounding box plus 3 m per side; uniform heading in `[-pi,pi]` |
| Seed | `2004064 + case_id` |
| Neighbor selection | 16 nearest existing nodes by `dx^2+dy^2+(wrap(dtheta)/kappa_max)^2` |
| Edges | Independently compute both directed CC connections; retain collision-free ones |
| Query | Dijkstra minimizing total path length, after graph construction |
| Curvature | `tan(0.7)/2.8` per common vehicle |
| Curvature derivative | `0.5/(2.8*2.5)` m^-2, a common-vehicle adapter, not the paper's example value |
| Export | At most 0.05 m, exactly equal spacing within each direction phase, exact cusp samples |

Start-to-goal and goal-to-start connections are attempted first. No other planner supplies a seed, fallback, or substitute result. The whole graph is rebuilt and timed on every call; this does not measure offline multi-query PRM reuse. The time cap is checked between sample insertions, so an in-flight insertion can exceed it. The extra curvature-derivative limit implies the common steering-rate bound whenever speed remains below the common maximum, since `abs(phi_dot)=L*abs(v*kappa_s)/(1+(L*kappa)^2)`.

`+ccp/Edge.m` checks the complete vehicle rectangle. A convex support-plane lower bound on body-to-obstacle distance is computed by GJK. Between samples separated by arc length `ds`, every body point moves at most `(1+R*kappa_max)*ds/2` from its nearer sample, where `R` is the rear-axle-to-farthest-corner distance. An interval is accepted only if its lower distance bound exceeds this amount plus a 1e-10 m numerical tolerance. Initial spacing is at most 0.15 m, including primitive boundaries; unresolved intervals are bisected and conservatively rejected below 1e-5 m. Distances are capped at 0.5 m. Thus this is a conservative continuous-body separation check subject to floating-point geometry accuracy, rather than just checking obstacle vertices at isolated poses. Frozen obstacles are convex rectangles; arbitrary concave inputs would require convex decomposition.

## MATLAB use

```matlab
SetupCommonParking;
BuildCCSteer;                     % one-time build using mex -setup C++
result = RunPlanner('CC_PRM',3);   % exactly one case
evaluation = EvaluateResult(result,LoadCase(3));
```

Windows also supports `BuildCCSteer('C:/path/to/zig.exe')` with portable Zig 0.13. Binaries and compiler caches go to `tempdir/CommonParking/cc-steer/<arch>` or `COMMONPARKING_CC_DIR`, never the source folder. The planner otherwise uses base MATLAB; `TestCCSteer` additionally uses Optimization Toolbox for independent convex QP checks. The common evaluator needs its documented external AMPL/Ipopt runtime.

The result is a **path**. Curvature is converted to front-wheel steering angle; interval gear signs and cusp indices are explicit. Continuous-curvature geometry does not imply the evaluator will exactly track the path under its timing, acceleration and steering-rate constraints. Native and executed results are reported separately, including failed roadmap queries.

## Validation

`TestCCSteer` checks 640 random family/topological connections, 60 independent ODE integrations, endpoints, reverse traversal, curvature continuity across every cusp, derivative limits, shrinking topological paths, support-plane distances against 30 independent primal QPs, and a thin obstacle between coarse check poses. Release validation additionally checks each returned graph route and length, primitive-by-primitive ODE integration, exact cusps, uniform phase spacing, continuous separation and 0.005 m full-body SAT samples. Published results are one fixed-seed call per case; they are not a statistical success-rate experiment.

Published MAT files retain the complete path and control primitives, selected roadmap poses/edge lengths and all validation summaries. The unused full roadmap is omitted from those compact files; a fresh `RunPlanner` call returns it in diagnostics.
