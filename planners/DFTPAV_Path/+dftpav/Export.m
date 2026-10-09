function [path,native]=Export(detail,vehicle)
distance=[];poses=[];gear=[];cuts=1;offset=0;native=struct('t',[],'x',[],'y',[],'theta',[],'v',[],'phi',[],'a',[],'omega',[],'gear',[]);timeOffset=0;
for i=1:numel(detail.segments)
 seg=detail.segments{i};M=seg.M;maps=cell(M,1);lengths=zeros(M,1);parameter=(0:.005:1)';
 for j=1:M
  C=seg.coefficients((j-1)*6+(1:6),:);d=C(2:6,:).*(1:5)';r=conv(d(:,1),d(:,1))+conv(d(:,2),d(:,2));stationary=roots(flipud((1:8)'.*r(2:9)));
  stationary=real(stationary(abs(imag(stationary))<1e-9&real(stationary)>0&real(stationary)<1));probe=[0;1;stationary];minimum=min(vecnorm(dftpav.Basis(probe,1)*C,2,2));
  assert(minimum>1e-8,'DFTPAV:SingularCurve','Optimized geometry has an internal velocity singularity.');
  cumulative=[0;cumsum(dftpav.ArcLength(C,parameter(1:end-1),parameter(2:end)))];lengths(j)=cumulative(end);maps{j}=struct('u',parameter,'s',cumulative,'C',C,'minimum_parameter_speed',minimum);
 end
 starts=[0;cumsum(lengths)];s=linspace(0,starts(end),max(1,ceil(starts(end)/.05))+1)';q=zeros(numel(s),4);which=discretize(s,[-Inf;starts(2:end-1);Inf]);
 for j=1:M
  ids=find(which==j);map=maps{j};target=s(ids)-starts(j);u=interp1(map.s,map.u,target,'linear','extrap');bin=discretize(target,[-Inf;map.s(2:end-1);Inf]);left=map.u(bin);right=map.u(bin+1);base=map.s(bin);
  for iteration=1:8
   estimated=base+dftpav.ArcLength(map.C,left,u);speed=vecnorm(dftpav.Basis(u,1)*map.C,2,2);u=max(left,min(right,u-(estimated-target)./speed));
  end
  u(target<=1e-12)=0;u(target>=lengths(j)-1e-12)=1;q(ids,:)=dftpav.EvaluatePiece(map.C,u,seg.gear,seg.piece_duration,vehicle);
  tn=linspace(0,1,max(2,ceil(seg.piece_duration/.02))+1)';[qn,~,values]=dftpav.EvaluatePiece(map.C,tn,seg.gear,seg.piece_duration,vehicle);
  % Native diagnostic arrays intentionally retain duplicate joins: the two
  % sides of a nonzero-speed gear switch are distinct, not silently averaged.
  native.t=[native.t;timeOffset+(j-1+tn)*seg.piece_duration];native.x=[native.x;qn(:,1)];native.y=[native.y;qn(:,2)];native.theta=[native.theta;qn(:,3)]; %#ok<AGROW>
  for name={'v','phi','a','omega'},f=name{1};native.(f)=[native.(f);values.(f)];end
  native.gear=[native.gear;seg.gear*ones(size(tn))]; %#ok<AGROW>
 end
 q(:,3)=unwrap(q(:,3));if ~isempty(poses),q(:,3)=q(:,3)+2*pi*round((poses(end,3)-q(1,3))/(2*pi));poses=poses(1:end-1,:);distance=distance(1:end-1);end
 poses=[poses;q];distance=[distance;offset+s];gear=[gear;seg.gear*ones(numel(s)-1,1)];cuts(end+1,1)=numel(distance);offset=offset+s(end);timeOffset=timeOffset+seg.duration; %#ok<AGROW>
end
path=struct('s',distance,'x',poses(:,1),'y',poses(:,2),'theta',poses(:,3),'phi',atan(vehicle.lw*poses(:,4)),'gear',gear,'cusp_indices',cuts);
native.description='Native polynomial time law, including duplicate join samples and 0.05 m/s endpoint/cusp speed; diagnostic only. The standardized submission is the optimized geometric path.';
end
