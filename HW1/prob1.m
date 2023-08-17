
close all;
clear;
clc;

% Read a color image and get a gray image
f = imread('heaveng.tif');
[X,Y] = size(f); % 528 704

% Plot its histogram and cumulative distribution.
n = 0:255;

% Histogram H of f
H = imhist(f,256)';
Hm = max(H);

% Cumulative distribution of H  
cH = cumsum(H);
N = cH(end); % Or, N = numel(f) = X*Y

figure(1); imshow(f); title('\bfGiven image f'); %impixelinfo;
figure(2); plot(n,H/Hm,'r',n,cH/N,'b',n,n/255,'k:','linewidth',1.5);
axis([0,260,0,1]); xlabel('\bf Gray level n = 0:255');
title('\bfHistogram H of given image f and its cumulative distribution cH');
legend('H','cH','n/255', 'Location','Nw');

% Compute and display the histogram equalized image and plot its histogram.
[feq, Heq] = heq(f, 255); % Ho is the histogram of f, i.e., Ho=H
cHeq = cumsum(Heq);

% We may use histeq.m such as
%   feq1 = histeq(f,256);  Heq1 = imhist(feq1,256);
% feq and Heq are very similar to feq1 and Heq1, respectively but unequal
figure(3); plot(n,H/Hm,'r', n,Heq/Hm,'b'); hold on;
plot(n,cH/N,'r',n,cHeq/N,'b','linewidth',1.5); axis([0,260,0,1]);
legend('H','Heq','cH','cHeq','Location','Nw'); legend boxoff;
title('\bf H, cH, Heq, and cHeq');
figure(4); imshow(feq); title('\bf Histogram equalized image feq');
