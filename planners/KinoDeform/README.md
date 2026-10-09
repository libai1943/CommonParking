# Kinodynamic exploration trees with trajectory deformation

Florent Lamiraux, Etienne Ferre and Erwan Vallee, *Kinodynamic Motion Planning: Connecting Exploration Trees Using Trajectory Optimization Methods*, ICRA 2004, pp. 3987–3992. [Author manuscript](https://homepages.laas.fr/florent/publi/04icra-erwan.pdf).

This implementation reproduces the article's two-stage algorithm and its five-state car example. Numerical choices omitted from the article are disclosed below. It does not use another planner to supply or repair a path.

The implemented model is the paper's five-state car `[x,y,theta,v,phi]`, with physical acceleration and steering-rate inputs. Two input-shooting trees grow from the exact start and goal states. The goal tree integrates `-f` away from its fixed root; reversing that branch restores the original physical model and control values. Algorithm 1 selects the valid shot with the fewest existing nodes in its endpoint neighborhood, retaining its printed last-on-tie rule. It does not select the shot closest to the random target.

The paper's maximum-component metric is used with weights `[1,1,L,L/vmax,L]` and angular differences modulo `2*pi`. The larger reported connection tolerance `1/8` is retained. Unpublished choices are sixteen constant-input trials per expansion, uniformly sampled duration 0.4–1.6 s, a 0.5 metric-unit neighborhood, a 3 m padded workspace and ten-percent sampling of a random node of the opposite tree. All other random states are sampled uniformly in the workspace, heading, speed and steering bounds. The seed is `2004110 + case_id`. The two trees alternate.

When a close pair is found, the two complete root-to-node branches are alternately deformed by perturbing their inputs. The root of each branch stays fixed. The implementation differentiates the RK4 flow and propagates the paper's variational equation `E'=A*E+B*e`. The potential is the printed one-half squared endpoint difference plus the time integral of an obstacle potential. The article gives `1/d` only as an example and does not specify the test functions or line-search parameters. Here the obstacle term is the explicitly chosen finite-range function `U(q)=0.5*max(0,0.08-d(q))^2`, where `d` is a conservative full-body support-plane separation. This avoids exerting an obstacle-potential gradient beyond the selected clearance range. Collision and input/state bounds remain hard acceptance conditions.

The finite test functions span all piecewise-constant branch inputs. To condition endpoint motion, let `J` be their terminal sensitivity matrix and choose `H=J'*J+1e-6*I=R'*R`. The current test functions are transformed by `R^-1`; the paper's coefficients `lambda=-mu` then give the original-input direction `-H^-1*gradient`. This adaptive basis choice is an explicit implementation choice, not a value or algorithm claimed to appear in the paper. All directions, including the obstacle integral, use the same basis. Backtracking accepts only a feasible reintegration with Armijo decrease; no state interpolation or endpoint snapping is used to close a gap. If deformation stalls, tree exploration resumes, as specified by the article.

Finite budgets are 30,000 nodes, 180 seconds total, twenty connection attempts, and 400 alternating deformation iterations/30 seconds per attempt. A residual of at most `1e-8` in the paper's metric is required to join branches. Search and deformation failures remain failures. An additional refinement repeats the same deformation on a 0.01 s integration mesh to a `1e-10` join tolerance. It rejects a result if refinement fails, the exported join exceeds `1e-8`, or a complete-body check fails. The published input-shooting and deformation budgets are uniform across all cases; a native failure is not a claim that the benchmark task has no solution.

The collision checker uses the entire vehicle rectangle and a speed-times-curvature body-motion bound to certify between-sample clearance. The existing CC_PRM module is used only for support-plane distance queries, not as a steering connector or an alternative planner.

## Output, dependencies and validation

The result is a trajectory. Input-shooting branches are integrated with RK4; output states are sampled at intervals no larger than 0.01 s, including every native input switch and every exact zero-speed cusp. The physical controls are piecewise constant. At a switch, the stored control belongs to the following interval; the final stored control is zero. The goal branch is traversed backward in its construction parameter, with the same physical input signs. The two near-identical join samples are merged within the documented `1e-8` tolerance, whose actual residual is retained. The goal pose is never overwritten after forward integration: it is the fixed root of the backward-integrated branch. Start and goal speed and steering are zero.

Native solver success requires both tree connection and successful fine-mesh deformation; it does not claim global time optimality or a globally optimal potential. Full native integration and body-clearance checks are independent of the common execution evaluator. That evaluator retains its common vehicle bounds and collision/terminal metrics.

```matlab
SetupCommonParking;
BuildCCSteer;  % one-time shared distance-query module build
result = RunPlanner('KinoDeform',3);
evaluation = EvaluateResult(result,LoadCase(3));
```

The planner uses base MATLAB and the CC_PRM distance module. No Optimization Toolbox, Navigation Toolbox, AMPL or external NLP solver is used by this planner. The common evaluator has its own AMPL/Ipopt dependency. Original branch inputs, durations, roots, deformation history and explicit finite join residuals remain in the public diagnostics. Large exploratory trees are omitted from the compact published results; they are present in a fresh `RunPlanner` return. All planning and fine-refinement work is included in the reported wall time; the one-time distance-module build is excluded.

`TestKinoDeform` checks forward/backward RK4 derivatives, independent ODE integration, the active obstacle-potential gradient, terminal sensitivities, an exactly reachable perturbed two-branch connection, fixed endpoint states and a thin obstacle between otherwise clear samples. Release verification independently integrates every successful physical input sequence, checks the whole-body clearance at at most 0.005 m travel, repeats the continuous-body certificate, recomputes all exported fields and verifies every zero-speed cusp. Failed searches and failed execution optimizations are retained in the twelve-case table.
