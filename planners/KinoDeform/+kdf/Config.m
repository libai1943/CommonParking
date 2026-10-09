function o=Config(v)
o.seed=2004110;o.padding=3;o.seconds=180;o.maxNodes=30000;o.randomCommands=16;
o.duration=[.4,1.6];o.goalBias=.1;o.neighbor=.5;o.joinTolerance=1/8;o.finalTolerance=1e-8;
o.treeStep=.05;o.deformStep=.04;o.outputStep=.01;o.deformSeconds=30;o.maxDeform=400;o.maxAttempts=20;
o.potentialRange=.08;o.potentialWeight=1;o.fdStep=1e-5;o.margin=1e-8;o.minimumTime=1e-5;
o.scale=[1,1,v.lw,v.lw/v.vmax,v.lw];o.basisRegularizer=1e-3;
end
