function [U,h]=Extract(tree,id)
ids=[];while tree.parent(id)>0,ids=[id;ids];id=tree.parent(id);end %#ok<AGROW>
U=tree.u(ids,:);h=tree.duration(ids);
end
