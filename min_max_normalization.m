function output = min_max_normalization(input)
output = (input - min(input)) / (max(input) - min(input));