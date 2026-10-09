function [duals,certificates]=DualSeed(keys,q,c)
polys=parking.PolygonData(c);v=c.vehicle;body=[v.lw+v.lf,v.lb/2;v.lw+v.lf,-v.lb/2;-v.lr,-v.lb/2;-v.lr,v.lb/2];
G=[1 0;0 1;-1 0;0 -1];g=[v.lw+v.lf;v.lb/2;v.lr;v.lb/2];duals=cell(size(keys,1),1);certificates=duals;
for m=1:size(keys,1)
 k=keys(m,1);type=keys(m,2);i=keys(m,3);j=keys(m,4);R=[cos(q.theta(k)) -sin(q.theta(k));sin(q.theta(k)) cos(q.theta(k))];t=[q.x(k);q.y(k)];
 if type==1,point=R*body(j,:)'+t;A=polys.A{i};b=polys.b{i};p=polys.vertices{i};else,point=R'*(polys.vertices{i}(j,:)'-t);A=G;b=g;p=body;end
 [duals{m},certificates{m}]=tpckc.PointDual(point,A,b,p);
end
end
