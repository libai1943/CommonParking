function [G,A]=Residual(z,N,vehicle,circles)
% Exact nonlinear equalities and sparse analytic Jacobian, Eqs. (6)-(10).
% z=[x;y;theta;v;phi;last_disc_x;last_disc_y;tf], columnwise over N states.
q=reshape(z(1:7*N),N,7);tf=z(end);h=tf/(N-1);i=(1:N-1)';
x=q(:,1);y=q(:,2);theta=q(:,3);v=q(:,4);phi=q(:,5);
offset=vehicle.length-vehicle.length/(2*circles)-vehicle.lr;
G=[diff(x)-h*v(i).*cos(theta(i));diff(y)-h*v(i).*sin(theta(i)); ...
 diff(theta)-h*v(i).*tan(phi(i))/vehicle.lw; ...
 q(:,6)-x-offset*cos(theta);q(:,7)-y-offset*sin(theta)];
if nargout<2,return;end
rows=zeros(0,1);columns=rows;values=rows;K=N-1;D=7*N+1;
for axis=1:3
 row=(axis-1)*K+i;state=(axis-1)*N+i;
 add(row,state+1,ones(K,1));add(row,state,-ones(K,1));
end
add(i,3*N+i,-h*cos(theta(i)));add(i,2*N+i,h*v(i).*sin(theta(i)));add(i,repmat(D,K,1),-v(i).*cos(theta(i))/K);
add(K+i,3*N+i,-h*sin(theta(i)));add(K+i,2*N+i,-h*v(i).*cos(theta(i)));add(K+i,repmat(D,K,1),-v(i).*sin(theta(i))/K);
add(2*K+i,3*N+i,-h*tan(phi(i))/vehicle.lw);add(2*K+i,4*N+i,-h*v(i)./(vehicle.lw*cos(phi(i)).^2));
add(2*K+i,repmat(D,K,1),-v(i).*tan(phi(i))/(vehicle.lw*K));
j=(1:N)';add(3*K+j,5*N+j,ones(N,1));add(3*K+j,j,-ones(N,1));add(3*K+j,2*N+j,offset*sin(theta));
add(3*K+N+j,6*N+j,ones(N,1));add(3*K+N+j,N+j,-ones(N,1));add(3*K+N+j,2*N+j,-offset*cos(theta));
A=sparse(rows,columns,values,numel(G),D);
    function add(r,c,vv)
        rows=[rows;r];columns=[columns;c];values=[values;vv]; %#ok<AGROW>
    end
end
