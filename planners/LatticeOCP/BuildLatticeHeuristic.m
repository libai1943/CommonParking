function lookup=BuildLatticeHeuristic(libraryFile,outputFile)
% Exact obstacle-free graph costs. Stopped Dijkstra labels are never exported.
lattice.EnsureNative();stored=load(libraryFile,'library');library=stored.library;
edges=lattice.EdgeMatrix(library);radius=round(library.options.heuristicSideLength/(2*library.options.grid));
tic;[cost,statistics]=lattice_graph_mex('lookup',edges,16,radius,128);elapsed=toc;
lookup=struct('schema','CommonParking-LatticeOCP-heuristic-1','cost',cost,'radius',radius,'statistics',statistics, ...
    'preparation_time_s',elapsed,'primitive_signature',library.signature);
save(outputFile,'lookup','-v7');fprintf('Generated complete %d-by-%d-by-16-by-16 heuristic in %.3f seconds.\n',2*radius+1,2*radius+1,elapsed);
end
