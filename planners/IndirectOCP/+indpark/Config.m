function o=Config()
o.nodes=500;o.timeWeight=1;o.trackWeights=[10;10;10];o.obstacleWeight=1000;o.smoothing=.1;
o.barriers=[.1 .001];o.maxIterations=180;o.maxSeconds=180;o.tolerance=1e-7;o.maxBacktracks=24;
o.search=hacg.Config().search;
end
