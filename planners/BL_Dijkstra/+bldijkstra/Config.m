function o=Config()
% Published Section 6 search, with two gear-indexing arrays (Claim 2).
o.resolutionExponent=8;o.padding=5;o.maximumSeconds=180;
o.maximumNodes=2^(3*o.resolutionExponent+1); % Full pair of direction-indexing arrays.
o.outputSpacing=.05;
end
