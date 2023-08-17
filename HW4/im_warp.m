function out = im_warp(in, H, R)

% Homograph from 'out' to 'in'
Hinv = pinv(H);

% Size of image 'in'
H_in = size(in,1);
W_in = size(in,2);

% Size of image 'out'
H_out = R.ImageSize(1);
W_out = R.ImageSize(2);

% Initialize image 'out'
out = uint8(zeros(H_out, W_out, 3));

% Compute image 'out'
for i = 1:W_out
    for j = 1:H_out
        % Coordinates of output point
        x = (i-1)*R.PixelExtentInWorldX + R.XWorldLimits(1);
        y = (j-1)*R.PixelExtentInWorldY + R.YWorldLimits(1);

        %======= IMPLEMENT HERE!! =======%
        % Transform coordinates using the inverse homography
        p = Hinv * [x; y; 1];

        % Convert to homogeneous coordinates
        p = p ./ p(3);

        % Map the point to the input image
        x_in = p(1);
        y_in = p(2);

        % Check if the mapped point is within the input image bounds
        if x_in >= 1 && x_in <= W_in && y_in >= 1 && y_in <= H_in
            % Perform bilinear interpolation
            x_floor = floor(x_in);
            y_floor = floor(y_in);
            x_ceil = ceil(x_in);
            y_ceil = ceil(y_in);

            % Interpolation weights
            wx = x_in - x_floor;
            wy = y_in - y_floor;

            % Retrieve pixel values from the input image
            pixel_tl = double(in(y_floor, x_floor, :));
            pixel_tr = double(in(y_floor, x_ceil, :));
            pixel_bl = double(in(y_ceil, x_floor, :));
            pixel_br = double(in(y_ceil, x_ceil, :));

            % Perform bilinear interpolation
            pixel_interp = (1 - wy) * ((1 - wx) * pixel_tl + wx * pixel_tr) + ...
                wy * ((1 - wx) * pixel_bl + wx * pixel_br);

            % Assign the interpolated pixel value to the output image
            out(j, i, :) = uint8(pixel_interp);
        end
        %================================%

    end
end

