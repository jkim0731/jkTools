function corval = nancorr(x,y)
% calculate rho (correlation coefficient) with ignoring NaNs in each of the
% input vector

% 2020/04/08 JK

finiteInd = intersect(find(isfinite(x)), find(isfinite(y)));
corval = corr(x(finiteInd), y(finiteInd));