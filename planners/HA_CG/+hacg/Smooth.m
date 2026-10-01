function [raw,details]=Smooth(c,seed,opt)
% Dolgov et al. (2010) Sec.3.1-3.4: coarse point CG, collision anchoring, fine fixed-node CG.
opt=parking.Options(struct('coarseSpacing',0.4,'fineSpacing',0.08, ...
    'maxIterations',600,'gradientTolerance',1e-3,'stepTolerance',1e-7, ...
    'maxAnchorPasses',12,'wObstacle',0.15,'obstacleRange',1.1, ...
    'wCurvature',3000,'wCurvatureMin',0,'wSmooth',0.5, ...
    'wVoronoi',0.2,'voronoiRange',4,'alpha',1, ...
    'curvatureTarget',c.vehicle.kappa_max),opt);
clock=tic;field=hacg.VoronoiField(c);t=c.task;start=[t.x0 t.y0 t.theta0];
[q,d,scoarse]=parking.ResampleByGear(start,seed.primitives,opt.coarseSpacing);
[coarse,coarseInfo]=stage(q,d,scoarse,false(size(q,1),1),seed,'coarse');
[q,d,~,ss]=parking.SamplePrimitives(start,coarse.primitives,opt.fineSpacing);
s=[0;cumsum(abs(ss))];boundaries=[0;cumsum(abs(coarse.primitives(:,1)))];fixed=false(size(s));
for a=boundaries', [~,j]=min(abs(s-a));fixed(j)=true;end
opt.wCurvatureMin=1;opt.wObstacle=0;opt.wVoronoi=0;opt.wSmooth=0;
[raw,fineInfo]=stage(q,d,s,fixed,coarse,'fine');
details=struct('reference','Dolgov et al. (2010) Sec.3, Eq.(1),(2),(6)', ...
    'cg_variant','handwritten PR+ with Armijo backtracking', ...
    'coarse',coarseInfo,'fine',fineInfo,'runtime',toc(clock),'options',opt, ...
    'representation_adapter','tangent-matched biarcs, with exact arc replay and conservative body checks');
    function [result,info]=stage(reference,gear,arcS,fixed,fallback,name)
        n=size(reference,1);cuts=[1;n];
        for cut=cuts',fixed(max(1,cut-2):min(n,cut+2))=true;end
        P=reference(:,1:2);runs=cell(0,1);accepted=false;totalMoved=0;diagnostics=cell(0,1);
        for pass=1:opt.maxAnchorPasses
            free=repmat(~fixed,1,2);baseP=P;baseP(fixed,:)=reference(fixed,1:2);
            if any(free,'all')
                [z,cg]=hacg.ConjugateGradient(@objective,baseP(free),opt);P=baseP;P(free)=z;
            else
                P=baseP;cg=struct('exitflag',3,'message','all_states_anchored','iterations',0);
            end
            runs{end+1}=cg; %#ok<AGROW>
            Q=[P reference(:,3)];A=P(2:end-1,:)-P(1:end-2,:);B=P(3:end,:)-P(2:end-1,:);
            la=max(vecnorm(A,2,2),1e-9);lb=max(vecnorm(B,2,2),1e-9);
            tangent=gear(1:end-1).*A.*lb./la+gear(2:end).*B.*la./lb;
            angle=atan2(tangent(:,2),tangent(:,1));Q(2:end-1,3)=angle;
            Q(fixed,3)=reference(fixed,3);
            [pp,bad,pieces]=parking.BiarcLift(Q,gear,c.vehicle.kappa_max);
            for e=1:n-1
                if fixed(e)&&fixed(e+1)
                    pieces{e}=parking.SlicePrimitives(fallback.primitives,arcS(e),arcS(e+1));bad(e)=false;
                end
            end
            pp=vertcat(pieces{:});
            diagnostics{end+1}=struct('bad_biarcs',find(bad),'candidate',Q,'primitives',pp,'fixed',fixed); %#ok<AGROW>
            for e=1:n-1
                if bad(e)||isempty(pieces{e}),bad(e)=true;continue;end
                samples=parking.SamplePrimitives(Q(e,:),pieces{e},0.035);
                if ~parking.FootprintClearance(samples,c,0.025),bad(e)=true;end
            end
            if ~any(bad)
                trial=fallback;trial.primitives=pp;trial.success=true;
                report=parking.ValidateSolution(c,trial);
                if report.feasible
                    result=trial;accepted=true;totalMoved=max(vecnorm(P-reference(:,1:2),2,2));break;
                end
                bad(:)=true;
            end
            before=fixed;
            for e=find(bad)',fixed(max(1,e-1):min(n,e+2))=true;end
            P(fixed,:)=reference(fixed,1:2);
            if isequal(before,fixed),break;end
        end
        if ~accepted,result=fallback;end
        info=struct('name',name,'passes',numel(runs),'runs',{runs},'accepted',accepted, ...
            'anchored_fraction',mean(fixed),'max_node_displacement',totalMoved, ...
            'returned_input',~accepted||totalMoved<1e-5,'diagnostics',{diagnostics});
        function [f,g]=objective(z)
            Z=baseP;Z(free)=z;[f,G]=hacg.Objective(Z,gear,c,field,opt);
            g=G(free);
        end
    end
end
