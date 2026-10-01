function PlotCase(c,ax,result,compact)
% TPCAP convention: gray obstacles, green initial and red terminal footprint.
if nargin<2||isempty(ax),figure;ax=axes;end
if nargin<3,result=[];end
if nargin<4,compact=false;end
hold(ax,'on'); axis(ax,'equal'); box(ax,'on'); grid(ax,'on');
ax.FontName='Arial';ax.FontSize=11;ax.GridAlpha=0.12;ax.Layer='top';
t=c.task; start=[t.x0 t.y0 t.theta0];goal=[t.xf t.yf t.thetaf];
points=[];
for j=1:c.obstacle.num_obs
    o=c.obstacle.obs{j}; shade=[0.56 0.58 0.61];
    if isfield(o,'kind') && ~strcmp(o.kind,'parked_car'),shade=[0.32 0.35 0.38];end
    fill(ax,o.x,o.y,shade, ...
        'EdgeColor',[0.35 0.37 0.40],'LineWidth',0.8,'HandleVisibility','off');
    points=[points;o.x' o.y']; %#ok<AGROW>
end
if ~isempty(result)&&result.success
    q=result.poses;d=result.direction;
    for sg=[1 -1]
        xx=[];yy=[];
        mask=d==sg;beg=find(diff([false;mask;false])==1);last=find(diff([false;mask;false])==-1)-1;
        for j=1:numel(beg)
            run=beg(j):last(j)+1;
            xx=[xx q(run,1)' NaN];yy=[yy q(run,2)' NaN]; %#ok<AGROW>
        end
        if sg>0,color=[0.08 0.37 0.72];style='-';else,color=[0.85 0.38 0.08];style='--';end
        plot(ax,xx,yy,style,'Color',color,'LineWidth',1.8,'HandleVisibility','off');
    end
    points=[points;q(:,1:2)];
end
poses=[start;goal];colors=[0.05 0.55 0.24;0.85 0.13 0.18];h=gobjects(2,1);
for j=1:2
    p=parking.VehiclePolygon(poses(j,:),c.vehicle);points=[points;p]; %#ok<AGROW>
    h(j)=patch(ax,p(:,1),p(:,2),colors(j,:),'FaceAlpha',0.045, ...
        'EdgeColor',colors(j,:),'LineStyle','--','LineWidth',1.8);
    quiver(ax,poses(j,1),poses(j,2),1.6*cos(poses(j,3)),1.6*sin(poses(j,3)),0, ...
        'Color',colors(j,:),'LineWidth',2,'MaxHeadSize',0.55,'HandleVisibility','off');
    plot(ax,poses(j,1),poses(j,2),'.','Color',colors(j,:),'MarkerSize',13,'HandleVisibility','off');
end
lo=min(points)-0.7;hi=max(points)+0.7;
daspect(ax,[1 1 1]);xlim(ax,[lo(1) hi(1)]);ylim(ax,[lo(2) hi(2)]);
xlabel(ax,'x (m)');ylabel(ax,'y (m)');
if compact
    words=strsplit(c.name,' - ');
    if isfield(c.design,'nominal_slot_length')
        words{2}=sprintf('%s | nominal %.1f m',words{2},c.design.nominal_slot_length);
    end
    label={sprintf('Case %02d | %s',c.id,words{1}),words{2}};
else
    label=sprintf('Case %02d | %s',c.id,c.name);
end
title(ax,label,'FontWeight','bold','FontSize',12,'Interpreter','none');
if ~compact
    legend(ax,h,{'Start pose','Goal pose'},'Location','northeastoutside');
    subtitle(ax,sprintf('Start (%.3f, %.3f, %.1f deg)   Goal (%.3f, %.3f, %.1f deg)', ...
        start(1),start(2),start(3)*180/pi,goal(1),goal(2),goal(3)*180/pi),'FontSize',10);
end
% Do not call axis equal after setting limits: it can expand the view in tiles.
set(ax,'XLimMode','manual','YLimMode','manual','DataAspectRatio',[1 1 1]);
end
