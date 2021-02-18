function ci = clustering_index(data, group, varargin)
% Calculate everage clustering index from all data points. 
% Clustering index in each data point is calculated by 
% ( mean(between-group distances) - mean(within-group distance) ) / 
%   ( mean(between-group distances) + mean(within-group distance) )
% This value can range (-1, 1]. 

% Input:
%     data: n x p data, n data points in p dimensions. Hilbert space values
%     group: n x 1, each value showing corresponding groups. Each group has
%       the same integer.
%     varargin{1}: String. Distance calculation methods ('euclidean', 'cityblock', or 'manhattan' for now; 'cityblock' and 'manhattan' are the same). Default, 'euclidean'.
%     varargin{2}: String. 'parametric', 'nonparametric-all', or 'nonparametric-each'.
% 
% Output:
%     ci: average clustering index

% For 'Neural stretching in S1' project (2020)
% 2020/08/28 JK

% Updates:
% 2020/09/17 JK
%     Can take 'methods' variable, to use distance value or rank,
%     can use 'euclidean' or 'cityblock' (same as 'manhattan')
% 2020/12/28 JK
%     Consider when some of the data matrix is NaN

methodGroup = {'euclidean', 'cityblock', 'manhattan'};
paramGroup = {'parametric', 'nonparametric-all', 'nonparametric-each'};
if length(group) ~= numel(group)
    error('Input ''group'' should be a vector')
end

if size(data,1) ~= length(group)
    error('Input ''data'' should have n x p shape, n same as the length of ''group'' vector')
end

nanInd = find(isnan(sum(data,2)));
data(nanInd,:) = [];
group(nanInd,:) = [];

if nargin < 3
    method = 'euclidean';
    param = 'parametric';
elseif nargin >= 3
    methodTemp = varargin{1};
    flags = startsWith(methodGroup, methodTemp, 'ignorecase', true);
    if sum(flags) < 0
        error('3rd input should be distance calculation method: (''euclidean'', ''cityblock'', or ''manhattan'')')
    elseif sum(flags) > 1
        error('3rd input argument is ambiguous among: (''euclidean'', ''cityblock'', or ''manhattan'')')
    else
        mothod = methodGroup{find(flags)};
    end

    if nargin >= 4
        paramTemp = varargin{2};
        flags = startsWith(paramGroup, paramTemp, 'ignorecase', true);
        if sum(flags) < 0
            error('4th input should be either ''parametric'' or ''nonparametric''')
        elseif sum(flags) > 1
            error('4th input argument is ambiguous among: ''parametric'' or ''nonparametric''')
        else
            param = paramGroup{find(flags)};
        end
    end
end

ciAll = zeros(length(group),1);
if strcmp(param,'nonparametric-all')
    % Make distance rank matrix using all pairs
    % Calculate from each pair and then make it symmetric
    allDist = triu(pdist2(data,data, method)); % Leave upper triangle
    allDist(find(allDist==0)) = nan; % Nan lower triangle, including diagonals (because they are 0 in distance)
    [~,allSorti] = sort(allDist(:),'ascend'); % Sort once, collect the indices
    [~,allSortii] = sort(allSorti,'ascend'); % Sort the indices
    distRankUp = reshape(allSortii, size(allDist)); % Upper triangular matrix of the distance rank
    distRankUp(find(isnan(distRankUp))) = 0; 
    distRankAll = distRankUp + distRankUp';
    distRankAll(find(distRankAll==0)) = nan; % Diagonals are now nan again
elseif strcmp(param,'nonparametric-each')
    % make distance rank matrix from each row
    allDist = pdist2(data,data, method);
    diagNan = 1-eye(size(allDist,1));
    diagNan(find(diagNan==0)) = nan;
    allDist = allDist .* diagNan; % Make diagonals NaN
    [~,rowSorti] = sort(allDist,2,'ascend');
    [~,distRankRow] = sort(rowSorti,2,'ascend');
    distRankRow = distRankRow .* diagNan;
end
for ci = 1 : length(ciAll)
    tempGroup = group(ci);
    betweenInd = find(group ~= tempGroup);
    withinInd = setdiff(find(group == tempGroup), ci);
    
    if strcmp(param,'nonparametric-all')
        betweenDist = mean(distRankAll(ci,betweenInd));
        withinDist = mean(distRankAll(ci,withinInd));
        if any(isnan(betweenDist) + isnan(withinDist))
            error('Calculation of distance rank of all pairs is wrong.')
        end
    elseif strcmp(param,'nonparametric-each')
        betweenDist = mean(distRankRow(ci,betweenInd));
        withinDist = mean(distRankRow(ci,withinInd));
        if any(isnan(betweenDist) + isnan(withinDist))
            error('Calculation of distance rank of all pairs is wrong.')
        end
    else
        betweenDist = mean(pdist2(data(ci,:), data(betweenInd,:), method));
        withinDist = mean(pdist2(data(ci,:), data(withinInd,:), method));
    end
    ciAll(ci) = (betweenDist - withinDist) / (betweenDist + withinDist);
end

ci = mean(ciAll);

