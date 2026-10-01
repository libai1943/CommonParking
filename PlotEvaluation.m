function fig = PlotEvaluation(result,evaluation,c)
%PLOTEVALUATION Optional inspection; figures do not contribute to metrics.
SetupCommonParking();fig=figure('Color','w','Name',sprintf('%s / case %02d',result.planner,c.id));
ax=axes(fig);hold(ax,'on');axis(ax,'equal');box(ax,'on');
for j=1:c.obstacle.num_obs
    ob=c.obstacle.obs{j};patch(ax,ob.x,ob.y,[.34 .36 .40],'EdgeColor',[.2 .21 .24],'LineWidth',.5);
end
if ~isempty(result.(result.kind))
    p=result.(result.kind);plot(ax,p.x,p.y,'Color',[.18 .4 .8],'LineWidth',.9,'DisplayName','Submitted');
end
if isfield(evaluation.debug,'execution')
    z=evaluation.debug.execution;plot(ax,z.x,z.y,'Color',[.83 .28 .16],'LineWidth',.8,'DisplayName','Tracked');
    distance=[0;cumsum(hypot(diff(z.x),diff(z.y)))];ids=1;last=0;
    for k=2:numel(distance)
        if distance(k)-last>=.75,ids(end+1)=k;last=distance(k);end %#ok<AGROW>
    end
    for k=ids
        color=[.2 .45 .8];if evaluation.debug.collision_mask(k),color=[.9 .05 .15];end
        body=parking.VehiclePolygon([z.x(k) z.y(k) z.theta(k)],c.vehicle);
        patch(ax,body(:,1),body(:,2),color,'FaceAlpha',.055,'EdgeAlpha',.45,'LineWidth',.35);
    end
end
poses=[c.task.x0 c.task.y0 c.task.theta0;c.task.xf c.task.yf c.task.thetaf];colors=[.1 .62 .31;.85 .18 .22];
for j=1:2
    q=poses(j,:);body=parking.VehiclePolygon(q,c.vehicle);
    patch(ax,body(:,1),body(:,2),colors(j,:),'FaceAlpha',.10,'EdgeColor',colors(j,:),'LineWidth',.8);
    quiver(ax,q(1),q(2),1.7*cos(q(3)),1.7*sin(q(3)),0,'Color',colors(j,:),'LineWidth',.9,'MaxHeadSize',.6);
end
xlabel(ax,'x (m)');ylabel(ax,'y (m)');title(ax,sprintf('%s | case %02d | %s',result.planner,c.id,evaluation.code),'Interpreter','none');
end
