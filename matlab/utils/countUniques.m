function [max_count_val, unique_counts, unique_vals] = countUniques(data)

unique_vals = unique(data(:));
unique_counts = zeros(size(unique_vals));
for i = 1:length(data),
  j = find(unique_vals == data(i), 1, 'first');
  unique_counts(j) = unique_counts(j) + 1;
end
[~, j_max] = max(unique_counts);
max_count_val = unique_vals(j_max);

end
