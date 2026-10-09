function result = SolveCase(name,c)
% Dispatch only released methods. Native exceptions become explicit failures.
name = upper(string(name));
switch name
    case {"HA_CG","HA+CG"}
        canonical = 'HA_CG'; planner = @hacg.Plan;kind='path';
    case "STC"
        canonical = 'STC'; planner = @stc.Plan;kind='trajectory';
    case "LIOM"
        canonical = 'LIOM'; planner = @liom.Plan;kind='trajectory';
    case "LATTICEOCP"
        canonical = 'LatticeOCP'; planner = @lattice.Plan;kind='path';
    case "SLIFS"
        canonical = 'SLiFS'; planner = @slifs.Plan;kind='trajectory';
    case "LAUMONDRS"
        canonical = 'LaumondRS'; planner = @laumond.Plan;kind='path';
    case "LAMIRAUXSMOOTH"
        canonical = 'LamirauxSmooth'; planner = @lamiraux.Plan;kind='path';
    case "CPRM"
        canonical = 'CPRM'; planner = @cprm.Plan;kind='path';
    case "DL_IAPS_PJSO"
        canonical = 'DL_IAPS_PJSO'; planner = @dliaps.Plan;kind='trajectory';
    case "ETA3"
        canonical = 'Eta3'; planner = @eta3.Plan;kind='path';
    case "TEB"
        canonical = 'TEB'; planner = @teb.Plan;kind='trajectory';
    case "SE2_NMPC"
        canonical = 'SE2_NMPC'; planner = @se2mpc.Plan;kind='trajectory';
    case "WGRRT"
        canonical = 'WGRRT'; planner = @wgrrt.Plan;kind='path';
    case "OSEHS"
        canonical = 'OSEHS'; planner = @osehs.Plan;kind='path';
    case "VPF"
        canonical = 'VPF'; planner = @vpf.Plan;kind='trajectory';
    case "RITP"
        canonical = 'RITP'; planner = @ritp.Plan;kind='trajectory';
    case "ANYTIMEPSRO"
        canonical = 'AnytimePSRO'; planner = @psro.Plan;kind='trajectory';
    case "DPGRID"
        canonical = 'DPGrid'; planner = @dpgrid.Plan;kind='path';
    case "BL_DIJKSTRA"
        canonical = 'BL_Dijkstra'; planner = @bldijkstra.Plan;kind='path';
    case "GRAPHBELLMAN"
        canonical = 'GraphBellman'; planner = @gbellman.Plan;kind='path';
    case "SMOOTHBIRRT"
        canonical = 'SmoothBiRRT'; planner = @sfrrt.Plan;kind='path';
    case "DUBINSGRID"
        canonical = 'DubinsGrid'; planner = @dgrid.Plan;kind='path';
    case "BICCHITANGENTS"
        canonical = 'BicchiTangents'; planner = @btangent.Plan;kind='path';
    case "TPCKC"
        canonical = 'TPCKC'; planner = @tpckc.Plan;kind='trajectory';
    case "DFTPAV_PATH"
        canonical = 'DFTPAV_Path'; planner = @dftpav.Plan;kind='path';
    case "HJBA"
        canonical = 'HJBA'; planner = @hjba.Plan;kind='path';
    case "HYPERPLANEOCP"
        canonical = 'HyperplaneOCP'; planner = @hpocp.Plan;kind='trajectory';
    case "BIRRT_HC"
        canonical = 'BiRRT_HC'; planner = @bihc.Plan;kind='path';
    case "TRIANGLEAREA"
        canonical = 'TriangleArea'; planner = @triangle.Plan;kind='trajectory';
    case "BOMP"
        canonical = 'BOMP'; planner = @bomp.Plan;kind='trajectory';
    case {"H_OBCA","OBCA"}
        canonical = 'H_OBCA'; planner = @hobca.Plan;kind='trajectory';
    case "BIRRT_HCR"
        canonical='BiRRT_HCR';planner=@bhcr.Plan;kind='path';
    case "CC_PRM"
        canonical='CC_PRM';planner=@ccp.Plan;kind='path';
    case "RTR_TTS"
        canonical = 'RTR_TTS'; planner = @rtr.Plan;kind='path';
    case "TDR_OBCA"
        canonical='TDR_OBCA';planner=@tdr.Plan;kind='trajectory';
    otherwise
        error('CommonParking:UnknownPlanner','Unknown released planner: %s',name);
end
result = cp.EmptyResult(canonical,c.id,kind);
try
    result = planner(c);
catch problem
    result.status.message = problem.message;
    result.status.code = 'planner_exception';
    result.diagnostics.exception = getReport(problem,'extended','hyperlinks','off');
end
end
