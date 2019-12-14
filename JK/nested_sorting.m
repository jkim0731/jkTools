function sorti = nested_sorting(inMat, inds)
% takes an input matrix with multiple columns
% min-max normalize each row
% sorting scheme: 
%     First by max order (grouping by the rows having the first element (column) as the largest, and then second element, ...)
%     Then within each group, by max order within the group.
%     Recursively run, until only one column is left.
%     Stitch the recursive results

% 2019/09/17 JK

sortDirection = 'ascend'; %or 'descend';
if size(inds,2) ~= 1
    inds = inds';
    if size(inds,2) ~=1
        error('indice should be a vector')
    end
end

if size(inMat,2) == 1
    [~, i] = sort(inMat, sortDirection);
    sorti = inds(i);
else
    tempCell = mat2cell(inMat, ones(size(inMat,1),1), size(inMat,2));
    tempMatCell = cellfun(@min_max_normalization, tempCell, 'uniformoutput', false);
    tempMat = cell2mat(tempMatCell);
    numGroup = size(tempMat,2);
    sortiCell = cell(numGroup,1);
    for i = 1 : numGroup
        row2sortTemp = find(tempMat(:,i)==1); % since it's min-max normalized. 
        if isempty(row2sortTemp)
            sortiCell{i} = [];
        else
            [~,sorti4row2sort] = sort(inMat(row2sortTemp,i), sortDirection);
            row2sort = row2sortTemp(sorti4row2sort);
            col2sort = setdiff(1:numGroup,i);
            tempsorti = nested_sorting(tempMat(row2sort,col2sort),inds(row2sort));
            if size(tempsorti,2) ~= 1
                tempsorti = tempsorti';
            end
            sortiCell{i} = tempsorti;
        end
    end
    sorti = cell2mat(sortiCell);
end

function output = min_max_normalization(input)
if max(input) == min(input)
    output = [1, zeros(1,length(input)-1)];
else
    output = (input - min(input)) / (max(input) - min(input));
    maxInds = find(output == 1);
    if length(maxInds) > 1
        noMaxInds = find(output < 1);
        nextMaxVal = max(output(noMaxInds));
        output(maxInds(2:end)) = (1+nextMaxVal)/2;
    end
end