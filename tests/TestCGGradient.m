function report = TestCGGradient()
SetupCommonParking();c=LoadCase(1);field=hacg.VoronoiField(c);
options=hacg.Config();opt=options.cg;opt.curvatureTarget=c.vehicle.kappa_max;
P=[0 0;.43 .12;.81 .29;1.21 .51;1.53 .59]+[c.task.x0 c.task.y0];
errors=zeros(4,1);
for j=1:4
    if j==3,opt.wObstacle=0;opt.wVoronoi=0;opt.wSmooth=0;opt.wCurvatureMin=1;end
    gear=ones(4,1);if mod(j,2)==0,gear(3:4)=-1;end
    [~,analytic]=hacg.Objective(P,gear,c,field,opt);numeric=zeros(size(P));
    for k=1:numel(P)
        a=P;b=P;a(k)=a(k)+1e-6;b(k)=b(k)-1e-6;
        numeric(k)=(hacg.Objective(a,gear,c,field,opt)-hacg.Objective(b,gear,c,field,opt))/2e-6;
    end
    errors(j)=norm(numeric-analytic,'fro')/max(1,norm(numeric,'fro'));
    assert(errors(j)<1e-6,'Analytic CG gradient disagrees with finite differences.');
end
report=struct('passed',true,'relative_errors',errors);disp(report);
end
