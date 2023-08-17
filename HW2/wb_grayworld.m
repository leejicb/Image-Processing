function [im_out] = wb_grayworld(im)
im_out = im;

% Calculate the mean values of the red, green, and blue channels
Red_avg = mean(mean(im(:,:,1)));
Green_avg = mean(mean(im(:,:,2)));
Blue_avg = mean(mean(im(:,:,3)));

% Calculate the average gray value of the image
Gray = (Red_avg + Green_avg + Blue_avg) / 3;

% Compute the scaling factors for each channel
Red_scale = Gray / Red_avg;
Green_scale = Gray / Green_avg;
Blue_scale = Gray / Blue_avg;

% Apply the scaling factors to the image
im_out(:,:,1) = im(:,:,1) * Red_scale;
im_out(:,:,2) = im(:,:,2) * Green_scale;
im_out(:,:,3) = im(:,:,3) * Blue_scale;



