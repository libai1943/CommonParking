function field=VoronoiField(c,ds)
% Shared polygon GVD construction used by search and smoothing.
if nargin<2,ds=.2;end
field=parking.VoronoiField(c,ds);
end
