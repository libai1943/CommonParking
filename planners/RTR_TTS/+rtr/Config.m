function o=Config(v)
o.kappa=v.kappa_max;o.sigma=v.wmax/(v.lw*v.vmax);
o.iterations=10000;o.globalSeconds=120;o.padding=3;o.seed=2017065;
o.localSamples=32;o.maximumLocalLength=1000;o.subdivisionCalls=1200;o.subdivisionDepth=24;o.subdivisionSeconds=180;
o.geometricClearance=.02;o.certificateDepth=20;o.translationStep=.3;o.rotationStep=pi/36;o.boundaryTolerance=1e-5;
o.checkStep=.15;o.minimumStep=1e-5;o.clearanceCap=.5;o.margin=0;o.outputStep=.05;
end
