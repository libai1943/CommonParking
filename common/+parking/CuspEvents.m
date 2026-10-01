function [indices,intervals]=CuspEvents(gear)
% A cusp may be one exact pose or a stopped steering dwell [entry,exit].
moving=find(gear~=0);intervals=zeros(0,2);
for k=2:numel(moving)
 a=moving(k-1);b=moving(k);
 if gear(a)~=gear(b),intervals(end+1,:)=[a+1 b];end %#ok<AGROW>
end
indices=unique(intervals(:));
end
