function [f,G,terms]=Objective(P,gear,c,field,opt)
% Dolgov et al. (2010) Eq.(1),(2),(6): obstacle, curvature, smoothness, Voronoi terms.
% Analytical gradients, including chain rule through all three curvature nodes.
n=size(P,1);G=zeros(size(P));terms=zeros(1,4);i=(2:n-1)';
if ~isempty(i)
    d1=gear(i-1);d2=gear(i);
    A=d1.*(P(i,:)-P(i-1,:));B=d2.*(P(i+1,:)-P(i,:));la=max(vecnorm(A,2,2),1e-7);lb=max(vecnorm(B,2,2),1e-7);
    % Paper Eq. (1)-(2): turning angle divided by the preceding chord.
    turn=atan2(A(:,1).*B(:,2)-A(:,2).*B(:,1),sum(A.*B,2));den=la;K=abs(turn)./den;
    excess=max(0,K-opt.curvatureTarget);w=2*opt.wCurvature*excess+2*opt.wCurvatureMin*K;
    terms(2)=opt.wCurvature*sum(excess.^2)+opt.wCurvatureMin*sum(K.^2);
    ga=sign(turn).*[A(:,2),-A(:,1)]./(la.^2.*den)-abs(turn).*A./(la.*den.^2);
    gb=sign(turn).*[-B(:,2),B(:,1)]./(lb.^2.*den);
    ga=d1.*w.*ga;gb=d2.*w.*gb;
    G=add(G,i-1,-ga);G=add(G,i,ga-gb);G=add(G,i+1,gb);
    dd=B-A;terms(3)=opt.wSmooth*sum(dd.^2,'all');z=2*opt.wSmooth*dd;
    G=add(G,i-1,d1.*z);G=add(G,i,-(d1+d2).*z);G=add(G,i+1,d2.*z);
end
% The fine pass disables both terms. Do not traverse obstacle polygons for
% a value and gradient whose weights are exactly zero.
if opt.wObstacle>0 || opt.wVoronoi>0
    [dist,dg]=parking.NearestObstacle(P,c);
end
if opt.wObstacle>0
    ex=max(0,opt.obstacleRange-dist);
    terms(1)=opt.wObstacle*sum(ex.^2);G=G-2*opt.wObstacle*ex.*dg;
end
if opt.wVoronoi>0&&~isempty(field.points)
    dv=zeros(n,1);vg=zeros(n,2);
    for begin=1:80:n
        ids=begin:min(begin+79,n);z=P(ids,:);pts=field.points;
        D=(z(:,1)-pts(:,1)').^2+(z(:,2)-pts(:,2)').^2;
        [value,j]=min(D,[],2);dv(ids)=sqrt(value);vg(ids,:)=(z-pts(j,:))./max(sqrt(value),1e-10);
    end
    d=max(dist,1e-6);e=max(dv,1e-6);active=dist>0&dist<opt.voronoiRange;
    alpha=opt.alpha;dm=opt.voronoiRange;aa=alpha./(alpha+d);bb=e./(d+e);cc=((d-dm)/dm).^2;
    rho=aa.*bb.*cc;rho(~active)=0;rho(dist<=0)=1;terms(4)=opt.wVoronoi*sum(rho);
    gd=-alpha./(alpha+d).^2.*bb.*cc-aa.*e./(d+e).^2.*cc+aa.*bb.*2.*(d-dm)/dm^2;
    ge=aa.*d./(d+e).^2.*cc;gd(~active)=0;ge(~active)=0;
    G=G+opt.wVoronoi*(gd.*dg+ge.*vg);
end
f=sum(terms);
end
function G=add(G,ids,z)
for dim=1:2,G(:,dim)=G(:,dim)+accumarray(ids,z(:,dim),[size(G,1),1]);end
end
