function discs = CoveringDiscs(vehicle,nx,ny)
% A circle circumscribes every cell of an nx-by-ny partition of the rectangle.
% Unlike adding discs only along the centreline, this converges to the actual
% vehicle rectangle in every direction as both partition counts increase.
dx=vehicle.length/nx;dy=vehicle.lb/ny;
[x,y]=ndgrid(-vehicle.lr+((1:nx)-.5)*dx,-vehicle.lb/2+((1:ny)-.5)*dy);
discs=struct('offsets',[x(:) y(:)],'radius',hypot(dx/2,dy/2), ...
    'count',nx*ny,'partition',[nx ny],'cell_size',[dx dy]);
end
