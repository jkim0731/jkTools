function out = sem(mat)
out = nanstd(mat) ./ sqrt(sum(isfinite(mat)));