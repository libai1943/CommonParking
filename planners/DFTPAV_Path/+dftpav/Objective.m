function [f,gradient,detail]=Objective(z,p)
% Analytic MINCO adjoint, including both sides of every movable cusp.
o=p.options;vehicle=p.caseData.vehicle;n=numel(p.segments);[T,dT]=dftpav.TimeMap(z(p.time_indices));
positions=[p.start(1:2);reshape(z(p.position_indices),n-1,2);p.goal(1:2)];headings=[p.start(3);z(p.angle_indices);p.goal(3)];
gradient=zeros(size(z));gp=zeros(n+1,2);gh=zeros(n+1,1);f=o.timeWeight*sum(T);pieces=cell(n,1);parts=[0 f 0];maxViolation=zeros(1,5);
u=(0:o.samplesPerPiece)'/o.samplesPerPiece;B=dftpav.Basis(u,0);B1=dftpav.Basis(u,1);B2=dftpav.Basis(u,2);quad=ones(size(u));quad([1 end])=.5;
body=[-vehicle.lr -vehicle.lb/2;vehicle.lw+vehicle.lf -vehicle.lb/2;vehicle.lw+vehicle.lf vehicle.lb/2;-vehicle.lr vehicle.lb/2];
for i=1:n
 seg=p.segments{i};M=seg.M;h=T(i)/M;eta=seg.gear;sys=seg.system;rhs=sys.B;
 v0=eta*o.gearSpeed*[cos(headings(i)) sin(headings(i))];v1=eta*o.gearSpeed*[cos(headings(i+1)) sin(headings(i+1))];
 rhs(sys.waypoint_rows,:)=reshape(z(seg.waypoint_indices),M-1,2);rhs(1,:)=positions(i,:);rhs(2,:)=h*v0;rhs(end-2,:)=positions(i+1,:);rhs(end-1,:)=h*v1;
 C=sys.A\rhs;gC=zeros(size(C));gTime=0;
 for j=1:M
  ids=6*(j-1)+(1:6);aC=C(ids,:);energy=sum(aC.*(sys.Q*aC),'all')/h^5;f=f+energy;parts(1)=parts(1)+energy;gC(ids,:)=2*sys.Q*aC/h^5;gTime=gTime-5*energy/h;
  xy=B*aC;vel=B1*aC/h;acc=B2*aC/h^2;r=sum(vel.^2,2);dotva=sum(vel.*acc,2);crossva=vel(:,1).*acc(:,2)-vel(:,2).*acc(:,1);
  if any(r<1e-24)||~isfinite(f),f=Inf;gradient(:)=0;detail=struct();return;end
  dotV=[acc(:,2) -acc(:,1)];dotA=[-vel(:,2) vel(:,1)];
  G=[r-vehicle.vmax^2,dotva.^2./r-vehicle.amax^2,crossva.^2./r-o.lateralAcceleration^2,crossva.^2./r.^3-vehicle.kappa_max^2];
  [loss,dl]=dftpav.Penalty(G,o.smoothing);maxViolation(1:4)=max(maxViolation(1:4),max(G,[],1));
  scale=h/o.samplesPerPiece*quad;weighted=dl.*(o.feasibilityWeight*scale);gx=zeros(size(xy));
  gv=weighted(:,1).*2.*vel + weighted(:,2).*(2*dotva./r.*acc-2*dotva.^2./r.^2.*vel) ...
    +weighted(:,3).*(2*crossva./r.*dotV-2*crossva.^2./r.^2.*vel) ...
    +weighted(:,4).*(2*crossva./r.^3.*dotV-6*crossva.^2./r.^4.*vel);
  ga=weighted(:,2).*(2*dotva./r.*vel)+weighted(:,3).*(2*crossva./r.*dotA)+weighted(:,4).*(2*crossva./r.^3.*dotA);
  penalty=o.feasibilityWeight*sum(scale.*sum(loss,2));
  for k=1:numel(u)
   cor=seg.corridors{(j-1)*o.samplesPerPiece+k};R=eta/sqrt(r(k))*[vel(k,:)' [-vel(k,2);vel(k,1)]];
   corners=body*R'+xy(k,:);Gobs=corners*cor.A'-cor.b';[lossObs,dlObs]=dftpav.Penalty(Gobs,o.smoothing);
   maxViolation(5)=max(maxViolation(5),max(Gobs,[],'all'));penalty=penalty+o.obstacleWeight*scale(k)*sum(lossObs,'all');gCorner=o.obstacleWeight*scale(k)*dlObs*cor.A;
   gx(k,:)=sum(gCorner,1);
   for e=1:4
    E=[body(e,1) body(e,2);-body(e,2) body(e,1)];offset=body(e,:)*R';
    gv(k,:)=gv(k,:)+eta/sqrt(r(k))*gCorner(e,:)*E'-dot(gCorner(e,:),offset)/r(k)*vel(k,:);
   end
  end
  f=f+penalty;parts(3)=parts(3)+penalty;gC(ids,:)=gC(ids,:)+B'*gx+B1'*gv/h+B2'*ga/h^2;
  gTime=gTime+penalty/h-sum(gv.*vel+2*ga.*acc,'all')/h;
 end
 Y=sys.A'\gC;gradient(seg.waypoint_indices)=reshape(Y(sys.waypoint_rows,:),[],1);
 gp(i,:)=gp(i,:)+Y(1,:);gp(i+1,:)=gp(i+1,:)+Y(end-2,:);
 gh(i)=gh(i)+dot(h*Y(2,:),eta*o.gearSpeed*[-sin(headings(i)) cos(headings(i))]);
 gh(i+1)=gh(i+1)+dot(h*Y(end-1,:),eta*o.gearSpeed*[-sin(headings(i+1)) cos(headings(i+1))]);
 gTime=gTime+dot(Y(2,:),v0)+dot(Y(end-1,:),v1);gradient(p.time_indices(i))=(gTime/M+o.timeWeight)*dT(i);
 pieces{i}=struct('coefficients',C,'duration',T(i),'piece_duration',h,'M',M,'gear',eta);
end
gradient(p.position_indices)=reshape(gp(2:end-1,:),[],1);gradient(p.angle_indices)=gh(2:end-1);
detail=struct('segments',{pieces},'objective_terms',parts,'native_time_s',sum(T),'maximum_sampled_constraint_values',maxViolation,'boundary_positions',positions,'boundary_headings',headings);
end
