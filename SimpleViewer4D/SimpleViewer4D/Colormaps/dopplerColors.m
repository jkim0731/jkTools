function gg = dopplerColors(m)
%DOPPLERCOLORS   QLab like Doppler colour colormap
%   DOPPLERCOLORS(M) returns an M-by-3 matrix containing a gray-scale colormap.
%   DOPPLERCOLORS, by itself, is the same length as the current figure's
%   colormap. If no figure exists, MATLAB creates one.
%
%   For example, to reset the colormap of the current figure:
%
%             colormap(dopplerColors)
%
%   See also HSV, HOT, COOL, BONE, COPPER, PINK, FLAG, 
%   COLORMAP, RGBPLOT.

%   Copyright 1984-2004 The MathWorks, Inc.
%   $Revision: 5.7.4.2 $  $Date: 2005/06/21 19:30:29 $

if nargin < 1, m = size(get(gcf,'colormap'),1); end
%g = (0:m-1)'/max(m-1,1);
%g = [g g g];
% g1 = ((0:m-1)'/max(m-1,1)).^2;
% g2 = (((0:m-1)'/max(m-1,1)-0.5)).^4;
% g3 = (1-(0:m-1)'/max(m-1,1)).^2;

rsize = floor(m/2)-1;
r=zeros(m,1);
r(1:rsize)= cos((0:rsize-1)/(rsize-1)*pi/2);


g = (((0:(m-1))-round(m/2))/(m-1)).^2;
g = g/max(g);

bsize = m - (floor(m/2)-1);
b=zeros(m,1);
b((bsize):end)= sin((0:(bsize-2))/(bsize-1)*pi/2).^(1/2);

 gg = [r(:) g(:) b(:)];
 
 gg = flipud(gg);
 
 %gg((round(m/2)-1):(round(m/2)+1),:)= zeros(3,3);