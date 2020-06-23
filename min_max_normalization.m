function output = min_max_normalization(input,varargin)
if nargin == 1
    output = (input - min(input)) ./ (max(input) - min(input));
elseif nargin > 1
    dim = varargin{1};
    output = (input - min(input,[],dim)) ./ (max(input, [], dim) - min(input, [], dim));
end