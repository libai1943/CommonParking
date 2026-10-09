function report=TestLatticeGraph()
% Known graph shortest paths validate the native heap, lookup and obstacles.
lattice.EnsureNative();edges=zeros(0,5);swept=cell(0,1);
for heading=1:2
    for delta=[1 0;-1 0;0 1;0 -1]'
        edges(end+1,:)=[heading heading delta' 1]; %#ok<AGROW>
        swept{end+1,1}=[(0:.25:1)'*delta',repmat(.1,5,1)]; %#ok<AGROW>
    end
    edges(end+1,:)=[heading 3-heading 0 0 1];swept{end+1,1}=[0 0 .1]; %#ok<AGROW>
end
[table,~]=lattice_graph_mex('lookup',edges,2,3,32);
[x,y]=ndgrid(-3:3);expected=abs(x)+abs(y);
assert(isequal(table(:,:,1,1),expected)&&isequal(table(:,:,1,2),expected+1));
[route,info]=lattice_graph_mex('search',edges,swept,[1 0 .2],[0 0 1],[2 0 1],[-4 4 -4 4],table,2,[2 10000 .01]);
assert(info(1)==1&&info(4)==4&&sum(edges(route,5))==4);
[~,info]=lattice_graph_mex('search',edges,swept,zeros(0,3),[0 0 1],[2 0 2],[-4 4 -4 4],table,2,[2 10000 .01]);
assert(info(1)==1&&info(4)==3);
fprintf('Lattice graph: exact toy lookup, obstacle detour and heading transition passed.\n');
report=struct('passed',true,'lookup_entries_checked',98,'obstacle_detour_cost',4,'heading_transition_cost',3);
end
