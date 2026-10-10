function phase=Phase(reference,gear,c,o)
% The author's polynomial family permits increasing degree on a failed fit.
% Both degrees use the same objective, endpoint and common physical checks.
history={};
for degree=o.degree:o.maximumDegree
 local=o;local.degree=degree;
 phase=ritp.SolvePhase(reference,gear,c,local);
 entry=struct('degree',degree,'success',phase.success,'code',phase.code);
 if isfield(phase,'maximum_steering'),entry.maximum_steering=phase.maximum_steering;end
 history{end+1}=entry; %#ok<AGROW>
 if phase.success,break;end
end
phase.degree_attempts=history;phase.degree=degree;
end
