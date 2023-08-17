function th = otsu(img)

% Compute the histogram 
counts = imhist(img);

% Normalize the histogram 
probs = counts / numel(img);
% Compute the cumulative distribution function 
cdf = cumsum(probs);
% Initialize the maximum between-class variance and the optimal threshold value
max_var = 0;
th = 0;

% Loop (0 to 255)
for i = 1:256
    % Compute the probabilities 
    class1_p = cdf(i);
    class2_p = 1 - class1_p;
    if class1_p > 0 && class2_p > 0
        % Compute the means
        class1_m = sum((0:i-1) .* probs(1:i)') / class1_p;
        class2_m = sum((i:255) .* probs(i+1:end)') / class2_p;
        % Compute the between-class variance 
        var_between = class1_p * class2_p * (class1_m - class2_m).^2;
        
        % update the optimal threshold
        if var_between > max_var
            max_var = var_between;
            th = i-1;
        end
    end
end

