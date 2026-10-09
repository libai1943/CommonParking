function [cost,grad,J,Q,t,E]=Potential(root,U,h,sgn,target,v,body,polygons,o)
if nargout>1,[Q,t,E]=kdf.Rollout(root,U,h,sgn,v,o.deformStep);else,[Q,t]=kdf.Rollout(root,U,h,sgn,v,o.deformStep);end
error=Q(end,:)-target;error(3)=atan2(sin(error(3)),cos(error(3)));
gap=cc_steer_mex('clearance',Q(:,1:3),body,polygons,o.potentialRange+1);p=max(0,o.potentialRange-gap);
cost=.5*sum(error.^2)+.5*o.potentialWeight*trapz(t,p.^2);
if nargout==1,return;end
J=E(:,:,end);grad=J'*error';weights=[diff(t);0]/2+[0;diff(t)]/2;
active=find(p>0);DQ=zeros(numel(active),5);
for k=1:3
 P=Q(active,1:3);P(:,k)=P(:,k)+o.fdStep;gp=cc_steer_mex('clearance',P,body,polygons,o.potentialRange+1);
 P(:,k)=P(:,k)-2*o.fdStep;gm=cc_steer_mex('clearance',P,body,polygons,o.potentialRange+1);
 DQ(:,k)=-o.potentialWeight*p(active).*(gp-gm)/(2*o.fdStep);
end
for n=1:numel(active),i=active(n);grad=grad+weights(i)*E(:,:,i)'*DQ(n,:)';end
end
