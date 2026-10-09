function [cost,terms]=Cost(arcs,weights)
arcs=arcs(abs(arcs(:,1))>1e-10,:);d=sign(arcs(:,1));lengths=abs(arcs(:,1));
terms=[sum(lengths),sum(lengths(d<0)),nnz(diff(d)),sum(lengths(abs(arcs(:,2))>1e-12))];cost=terms*weights(:);
end
