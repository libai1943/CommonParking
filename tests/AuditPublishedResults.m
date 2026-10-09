function report = AuditPublishedResults()
%AUDITPUBLISHEDRESULTS Check saved artifacts without rerunning any planner.
root=SetupCommonParking();cfg=BenchmarkConfig();cases=cell(12,1);
for id=1:12
    stored=load(fullfile(root,'cases',sprintf('Case%02d.mat',id)),'caseData');
    cases{id}=stored.caseData;cases{id}.vehicle=cfg.vehicle;
end
listing=dir(fullfile(root,'results'));reports={};total=0;
for k=1:numel(listing)
    name=listing(k).name;folder=fullfile(root,'results',name);
    source=fullfile(folder,'metrics.json');if ~isfile(source),continue;end
    records=jsondecode(fileread(source));assert(numel(records)==12);
    counts=zeros(1,4);codes=cell(12,1);
    for id=1:12
        [r,m,es]=LoadPublishedResult(name,id);
        if iscell(records),entry=records{id};else,entry=records(id);end
        assert(strcmp(r.planner,name)&&r.case_id==id&&ismember(r.kind,{'path','trajectory'}));
        assert(isequal(logical(r.status.success),logical(entry.planner_success)));
        assert(isequal(logical(es.success),logical(entry.evaluation_success))&&strcmp(es.code,entry.code));
        assert(abs(r.computation_time_s-entry.computation_time_s)<1e-10*max(1,r.computation_time_s));
        check=cpe.ValidateResult(r,cases{id},cfg);codes{id}=check.code;
        fields=fieldnames(m);
        for j=1:numel(fields)
            a=m.(fields{j});b=entry.metrics.(fields{j});if isempty(b),b=NaN;end
            assert((isnan(a)&&isnan(b))||abs(double(a)-double(b))<1e-10*max(1,abs(double(a))));
        end
        if es.success
            assert(check.valid,['Accepted artifact failed screening: ',name,' ',num2str(id),' ',check.message]);
            assert(m.collision_percent>=0&&m.collision_percent<=100&&m.execution_time_s>0);
            expected=10*m.control_effort_integral+10*m.steering_integral+5*m.gear_changes;
            assert(abs(m.smoothness_cost-expected)<1e-9*max(1,m.smoothness_cost));
            counts(3)=counts(3)+m.terminal_reached;
            counts(4)=counts(4)+(m.collision_percent==0);
        else
            assert(all(structfun(@(v)isscalar(v)&&isnan(v),m)));
        end
        counts(1)=counts(1)+r.status.success;counts(2)=counts(2)+es.success;total=total+1;
    end
    reports{end+1}=struct('planner',name,'cases',12,'planner_success',counts(1), ...
        'evaluation_success',counts(2),'terminal_reached',counts(3), ...
        'zero_collision_execution',counts(4),'screen_codes',{codes}); %#ok<AGROW>
    fprintf('AUDITED %s: %d native, %d evaluated, %d terminal\n',name,counts(1),counts(2),counts(3));
end
report=struct('passed',true,'planners',numel(reports),'results',total, ...
    'scope','Saved MAT loading (including chunk hashes), schema screening, MAT/JSON agreement, separate statuses and metric arithmetic. No planner or tracker is rerun.', ...
    'methods',{reports});
fprintf('AUDIT COMPLETE %d planners %d results\n',numel(reports),total);
end
