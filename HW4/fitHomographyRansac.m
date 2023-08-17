function [tform_1_2, indices, rmsErr] = ...
    fitHomographyRansac( ...
    keypts1, ...        % image 1 keypoints, size 4xN
    keypts2, ...        % image 2 keypoints, size 4xN
    max_iterations, ... % don't do more than this many iterations
    Ps, ...             % desired confidence level (prob of success); value from 0..1
    I1, I2 ...          % show images (for visualization only)
    )

% Output
tform_1_2 = [];
indices = [];
rmsErr = [];

% RANSAC parameters
s = 4;
t = 3.0;
proportion_of_outliers = 0.9; % We'll actually estimate this later from the data, but start with a worst case scenario assumption.

% Number of corresponding point pairs
N = size(keypts1,2);
if N < 4
    fprintf('Can''t fit a homography using less than 4 point pairs\n');
    return;
end

pts1 = keypts1(1:2,:);  % Get x,y coordinates from image 1, size is (2,N)
pts2 = keypts2(1:2,:);  % Get x,y coordinates from image 2, size is (2,N)

% Determine the required number of iterations
%======= IMPLEMENT HERE!! =======%
num_iterations = ceil(log(1 - Ps) / log(1 - (1 - proportion_of_outliers)^s));
%================================%
fprintf('Initial estimated number of iterations needed: %d\n', num_iterations);

% The number of Ransac trials
sample_count = 0;
num_inlier_max = -Inf;
while sample_count < num_iterations
    if sample_count > max_iterations
        break;
    end
    
    % Grab 4 matching points at random
    v = randperm(N);
    p1 = pts1(:,v(1:4));
    p2 = pts2(:,v(1:4));
    
    % Make sure the 4 points are not co-linear.  
    % Note: 3 points are colinear if (p3-p1)x(p2-p1) = 0
    q = [p1; ones(1,4)];
    if norm(cross(q(:,3)-q(:,1), q(:,2)-q(:,1))) < 1e-6 || ...
        norm(cross(q(:,4)-q(:,2), q(:,3)-q(:,2))) < 1e-6
        %disp('Image 1 points are degenerate:'), disp(p1);    
        continue;
    end
    q = [p2; ones(1,4)];
    if norm(cross(q(:,3)-q(:,1), q(:,2)-q(:,1))) < 1e-6 || ...
        norm(cross(q(:,4)-q(:,2), q(:,3)-q(:,2))) < 1e-6
        %disp('Image 2 points are degenerate:'), disp(p2);    
        continue;
    end
    
    % Try fitting a homography, that transforms p1 to p2. Matlab will
    % display a warning if the result is close to singular, so turn
    % warnings temporarily off.
    warning('off', 'all');
    try
        H = dlt(p1,p2);
        tform_1_2 = projective2d(H');
    catch
        continue;
    end
    warning('on', 'all');
    
    % Ok, we were able to fit a homography to this sample
    sample_count = sample_count + 1;
    
    % Use that homography to transform all pts1 to pts2
    pts2map = transformPointsForward(tform_1_2, pts1');
    
    % Look at the residual errors of all the points
    dp = (pts2map' - pts2);
    
    % Compute distance
    dist = sum(dp.^2, 1).^0.5;
    
    % number of inliers
    indices_inliers = dist < t;
    num_inlier = sum(indices_inliers);

    if num_inlier > num_inlier_max
        num_inlier_max = num_inlier;
        indices = indices_inliers;
        fprintf(' (Iteration %d): best so far with %d inliers\n', ...
            sample_count, num_inlier);
        
        % Show inliers
        if ~isempty(I1) && ~isempty(I2)
            figure(100), imshow([I1,I2],[]);
            o = size(I1,2) ;
            for i=1:size(pts1,2)
                x1 = pts1(1,i);
                y1 = pts1(2,i);
                x2 = pts2(1,i);
                y2 = pts2(2,i);

                if indices_inliers(i)
                    text(x1,y1,sprintf('%d',i), 'Color', 'g');
                    text(x2+o,y2,sprintf('%d',i), 'Color', 'g');
                else
                    text(x1,y1,sprintf('%d',i), 'Color', 'r');
                    text(x2+o,y2,sprintf('%d',i), 'Color', 'r');
                end
            end
        end
        pause
    end
    
    % Update the number of iterations required if we got a lot of inliers
    if 1 - num_inlier/N < proportion_of_outliers
        proportion_of_outliers = 1 - num_inlier/N;
        
        %======= IMPLEMENT HERE!! =======%
        num_iterations = ceil(log(1 - Ps) / log(1 - (1 - proportion_of_outliers)^s));
        %================================%
        fprintf(' (Iteration %d): New estimated number of iterations needed: %d\n', ...
            sample_count, num_iterations);
    end
end

%
fprintf('Final number of iterations used: %d\n', sample_count);
if sum(indices) < 4
    return % Couldn't find a fit
end
fprintf('Final calculated inlier fraction: %f\n', (num_inlier/N));

% Ok, refit homography using all the inliers
p1 = pts1(:,indices);
p2 = pts2(:,indices);
H = dlt(p1,p2);
tform_1_2 = projective2d(H');

% Determine the final residual error
p2map = transformPointsForward(tform_1_2, p1'); % Transform all p1 to p2

% Look at the residual errors of all the points
dp = (p2map' - p2);
rsq = dp(1,:).^2 + dp(2,:).^2; % residual squared distance errors

rmsErr = sqrt(sum(rsq)/length(rsq));
disp('RMS error: '), disp(rmsErr);

return

