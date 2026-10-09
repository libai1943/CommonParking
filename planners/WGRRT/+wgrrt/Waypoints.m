function [path,details]=Waypoints(c,o)
[path,details]=wgrrt.GeometricPath(c,o);if ~details.success,return;end
original=path;accepted=0;
for attempt=1:o.waypointShortcutTrials
 if size(path,1)<=2,break;end
 indices=sort(randperm(size(path,1),2));a=indices(1);b=indices(2);
 if b>a+1&&wgrrt.GeometricEdgeFree(path(a,:),path(b,:),c,o)
  path=[path(1:a,:);path(b:end,:)];accepted=accepted+1;
 end
end
path(:,3)=unwrap(path(:,3));details.original_path=original;details.shortcut_count=accepted;details.waypoints=size(path,1);
end
