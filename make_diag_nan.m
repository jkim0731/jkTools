function nanDiagMat = make_diag_nan(inputMat)
% Input matrix should be square
if length(size(inputMat)) ~=2
    error('Only works for 2 dimensional matrices.')
elseif size(inputMat,1) ~= size(inputMat,2)
    error('Input matrix should be square.')
end
n = size(inputMat,1);
nanMat = zeros(n) ./ (1-eye(n));
nanDiagMat = inputMat - nanMat;