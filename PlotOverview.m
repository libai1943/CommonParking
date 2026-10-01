function figureHandle = PlotOverview(results)
%PLOTOVERVIEW Three rows by four columns, thin strokes and translucent bodies.
% Supply a 12-element cell array of result structures in case order.
SetupCommonParking();assert(iscell(results)&&numel(results)==12);
figureHandle=figure('Color','w','Position',[40 40 1520 1070],'Visible','off');
layout=tiledlayout(figureHandle,3,4,'TileSpacing','compact','Padding','compact');
for caseId=1:12
    c=LoadCase(caseId);r=results{caseId};ax=nexttile(layout);hold(ax,'on');axis(ax,'equal');box(ax,'on');
    set(ax,'FontName','Arial','FontSize',9,'LineWidth',.45,'TickDir','out');
    for j=1:c.obstacle.num_obs
        ob=c.obstacle.obs{j};patch(ax,ob.x,ob.y,[.39 .41 .45],'EdgeColor',[.20 .22 .25],'LineWidth',.4);
    end
    if r.status.success
        p=r.(r.kind);distance=[0;cumsum(hypot(diff(p.x),diff(p.y)))];chosen=1;last=0;
        for k=2:numel(distance)
            if distance(k)-last>=.8,chosen(end+1)=k;last=distance(k);end %#ok<AGROW>
        end
        for k=chosen
            body=parking.VehiclePolygon([p.x(k) p.y(k) p.theta(k)],c.vehicle);
            patch(ax,body(:,1),body(:,2),[.16 .43 .76],'FaceAlpha',.045, ...
                'EdgeColor',[.28 .47 .64],'EdgeAlpha',.5,'LineWidth',.3);
        end
        plot(ax,p.x,p.y,'Color',[.1 .32 .70],'LineWidth',.8);
    end
    poses=[c.task.x0 c.task.y0 c.task.theta0;c.task.xf c.task.yf c.task.thetaf];colors=[.05 .58 .27;.82 .16 .16];
    for j=1:2
        q=poses(j,:);body=parking.VehiclePolygon(q,c.vehicle);
        patch(ax,body(:,1),body(:,2),colors(j,:),'FaceAlpha',.12,'EdgeColor',colors(j,:),'LineWidth',.7);
        quiver(ax,q(1),q(2),1.6*cos(q(3)),1.6*sin(q(3)),0,'Color',colors(j,:),'LineWidth',.8,'MaxHeadSize',.7);
    end
    title(ax,sprintf('Case %02d',caseId),'FontWeight','normal','FontSize',10);
    xlabel(ax,'x (m)');ylabel(ax,'y (m)');
    limits=axis(ax);axis(ax,limits+[-.4 .4 -.4 .4]);
end
end
