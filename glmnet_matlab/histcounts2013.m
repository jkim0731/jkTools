function bins = histcounts2013(input, ranges)
if isempty(input)
    bins = zeros(1,length(ranges)-1);
else
    tempbins = histc(input, ranges);
    bins = tempbins(1:end-1);
    if size(bins,1) > 1
        bins = bins';
    end
end
