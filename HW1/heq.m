function [feq, Heq] = heq(f, L)
% f: input image
% L: maximum intensity (for 8-bit gray-scale image, L = 255)
% feq: output image
% Heq: histogram of output image
% f_H: histogram of input image

feq = f;
Heq = zeros(1,L + 1);
f_H = zeros(L + 1, 1);
[height,width] = size(feq);

% histogram of input image without imhist function
for i = 1:height
    for j = 1:width
        %feq(i, j): uint8 -> Can't express 256 
        pixel_value = cast(feq(i, j), 'double');  
        f_H(pixel_value + 1) = f_H(pixel_value + 1) + 1;
    end
end

% Cumulative distribution of H
cH = cumsum(f_H);
cH = round(cH * L/cH(end)); % cH(end) = number of pixels

% output image
for i = 1:height    
    for j = 1:width
        %feq(i, j): uint8 -> Can't express 256 
        pixel_value = cast(feq(i, j), 'double');  
        feq(i,j) = cH(pixel_value + 1);
    end
end

% histogram of output image without imhist function
for i = 1:height
    for j = 1:width
        %feq(i, j): uint8 -> Can't express 256 
        pixel_value = cast(feq(i, j), 'double');  
        Heq(pixel_value + 1) = Heq(pixel_value +1) + 1;
    end
end
end