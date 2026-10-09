function ok=BranchFree(root,U,h,sgn,v,body,polygons,o)
ok=false;if any(abs(U(:,1))>v.amax+1e-10)||any(abs(U(:,2))>v.wmax+1e-10),return;end
q=root;
for i=1:size(U,1)
 [Q,t]=kdf.Rollout(q,U(i,:),h(i),sgn,v,o.deformStep);
 if ~kdf.Free(Q,t,U(i,:),sgn,body,polygons,v,o),return;end
 q=Q(end,:);
end
ok=true;
end
