function [p,bad,pieces]=BiarcLift(q,gear,kmax)
% Local representation adapter: exact tangent-matched arcs between poses.
% It is NOT an optimization algorithm or a substitute for the CG objective.
p=zeros(0,2);bad=false(size(gear));pieces=cell(numel(gear),1);
for i=1:numel(gear)
    d=gear(i);a=q(i,1:2);b=q(i+1,1:2);v=b-a;
    t0=d*[cos(q(i,3)) sin(q(i,3))];t1=d*[cos(q(i+1,3)) sin(q(i+1,3))];
    A=2*(1-dot(t0,t1));B=2*dot(v,t0+t1);C=-dot(v,v);
    if abs(A)<1e-12
        if B<=1e-12,bad(i)=true;continue;end
        u=-C/B;
    else
        u=2*(-C)/(B+sqrt(max(0,B*B-4*A*C)));
        if ~isfinite(u),u=(-B+sqrt(max(0,B*B-4*A*C)))/(2*A);end
    end
    mid=(a+b+u*(t0-t1))/2;
    [l1,k1,ok1]=arc(a,t0,mid);[l2,kr,ok2]=arc(b,-t1,mid);
    pp=[d*l1,d*k1;d*l2,-d*kr];pp(abs(pp(:,1))<1e-11,:)=[];
    if ~ok1||~ok2||isempty(pp)||any(~isfinite(pp),'all')||sum(abs(pp(:,1)))>10*norm(v)+1e-5
        bad(i)=true;continue;
    end
    near=abs(pp(:,2))<=kmax+1e-11;pp(near,2)=max(-kmax,min(kmax,pp(near,2)));
    replay=parking.SamplePrimitives(q(i,:),pp,inf);
    err=norm(replay(end,1:2)-q(i+1,1:2));ea=atan2(sin(replay(end,3)-q(i+1,3)),cos(replay(end,3)-q(i+1,3)));
    if err>1e-7||abs(ea)>1e-7||any(abs(pp(:,2))>kmax+1e-10),bad(i)=true;end
    p=[p;pp];pieces{i}=pp; %#ok<AGROW>
end
end
function [len,k,ok]=arc(a,t,b)
v=b-a;r2=dot(v,v);cross=t(1)*v(2)-t(2)*v(1);along=dot(t,v);ok=true;
if r2<1e-20,len=0;k=0;return;end
k=2*cross/r2;
if abs(k)<1e-9,len=norm(v);ok=along>=0;else,len=2*atan2(cross,along)/k;end
ok=ok&&len>=0;
end
