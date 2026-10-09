function o=Config(v)
% Common-vehicle limits; finite roadmap settings are disclosed adapter choices.
o.kappa=v.kappa_max;o.sigma=v.wmax/(v.lw*v.vmax);
o.nodes=2500;o.neighbours=16;o.seconds=60;o.padding=3;o.seed=2004064;
o.checkStep=.15;o.minimumStep=1e-5;o.clearanceCap=.5;o.margin=0;o.outputStep=.05;
end
