function pieces=SplinePieces(control,knots,direction)
breaks=unique(knots);pieces=repmat(cprm.Piece('cubic',direction),1,numel(breaks)-1);u=[0;1/3;2/3;1];V=[ones(4,1),u,u.^2,u.^3];
for j=1:numel(pieces)
 t=breaks(j)+(breaks(j+1)-breaks(j))*u;xy=cprm.Basis(t,knots,3)*control;pieces(j).coefficients=V\xy;
 [pieces(j).maximum_curvature,pieces(j).minimum_speed]=cprm.Extrema(pieces(j).coefficients);
end
end
