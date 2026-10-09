function score=SegmentScore(A,B,C,D)
% Appendix A: disjoint iff max(d1*d2,d3*d4)>0 in the printed criterion.
% It conservatively rejects disjoint collinear segments as well.
d1=cross2(A-C,D-C);d2=cross2(B-C,D-C);d3=cross2(C-A,B-A);d4=cross2(D-A,B-A);
score=max(d1.*d2,d3.*d4);
end
function z=cross2(a,b)
z=a(:,1).*b(:,2)-a(:,2).*b(:,1);
end
