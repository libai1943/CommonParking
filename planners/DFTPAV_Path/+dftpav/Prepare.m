function [z,problem]=Prepare(path,c,o)
seed=cp.EmptyResult('DFTPAV_seed',c.id,'path');seed.path=path;ref=cpe.MakeReference(seed,c);
n=numel(path.cusp_indices)-1;z=[];segments=cell(n,1);T=zeros(n,1);v=c.vehicle;
for i=1:n
 T(i)=ref.profiles{i}.tf;M=max(2,ceil(T(i)/o.pieceDuration));assert(M<=o.maximumPieces,'DFTPAV:Resource','Too many polynomial pieces.');
 waypointTime=ref.times(i)+(1:M-1)'/M*T(i);r=cpe.ReferenceAt(ref,waypointTime,v);points=[r.x r.y];ids=numel(z)+(1:numel(points));z=[z;points(:)]; %#ok<AGROW>
 [~,sys]=dftpav.MinimumJerk(zeros(M-1,2),[0 0],[0 0],[0 0],[0 0],1);
 sampleTime=ref.times(i)+(0:M*o.samplesPerPiece)'/(M*o.samplesPerPiece)*T(i);r=cpe.ReferenceAt(ref,sampleTime,v);
 corridor=cell(numel(sampleTime),1);for k=1:numel(sampleTime),corridor{k}=dftpav.Corridor([r.x(k) r.y(k) r.theta(k)],c,o);end
 segments{i}=struct('M',M,'waypoint_indices',ids,'gear',path.gear(path.cusp_indices(i)),'system',sys,'corridors',{corridor});
end
timeIndices=numel(z)+(1:n);tau=zeros(n,1);large=T>1;tau(large)=sqrt(2*T(large)-1)-1;tau(~large)=1-sqrt(2./T(~large)-1);z=[z;tau];
cuspPoints=[path.x(path.cusp_indices(2:end-1)) path.y(path.cusp_indices(2:end-1))];positionIndices=numel(z)+(1:numel(cuspPoints));z=[z;cuspPoints(:)];
angleIndices=numel(z)+(1:n-1);z=[z;path.theta(path.cusp_indices(2:end-1))];
problem=struct('segments',{segments},'time_indices',timeIndices,'position_indices',positionIndices,'angle_indices',angleIndices,'caseData',c,'options',o, ...
 'start',[path.x(1) path.y(1) path.theta(1)],'goal',[path.x(end) path.y(end) path.theta(end)],'initial_native_time_s',sum(T));
end
