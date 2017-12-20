%%%%
%imshow(imcomplement(bwimage))
%bwimage(x,y)
%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% use these code to change the map in mpc_test
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
close all;
clc;
% Import Image
image = imread('maptest.jpeg');

% Convert to grayscale and then black and white image based on arbitrary
% threshold.
grayimage = rgb2gray(image);
bwimage = grayimage < 195;

% Use black and white image as matrix input for binary occupancy grid
% Set the resolution to 2 cells per meter for this map.
map = robotics.BinaryOccupancyGrid(bwimage,20);

% display
figure
show(map)


%% the dimension of the robot can be assumed to be a circle with radius of 1 meters.
robotRadius = 0.2;

% Inflate the occupied positions by a given amount inflate(MAP, R) inflates each occupied position of the binary
% occupancy grid by at least R meters.
mapInflated = copy(map);
inflate(mapInflated,robotRadius);

figure
show(mapInflated)

%% save the map
save('test')






