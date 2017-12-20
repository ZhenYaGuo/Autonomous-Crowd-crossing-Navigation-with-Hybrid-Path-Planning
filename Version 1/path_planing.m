clear;
close all;
clc;
% Import Image
image = imread('testmap.jpeg');

% Convert to grayscale and then black and white image based on arbitrary
% threshold.
grayimage = rgb2gray(image);
bwimage = grayimage < 195;

% Use black and white image as matrix input for binary occupancy grid
% Set the resolution to 2 cells per meter for this map.
map = robotics.BinaryOccupancyGrid(bwimage,20);

% display
show(map)

%% the dimension of the robot can be assumed to be a circle with radius of 1 meters.
robotRadius = 0.2;

% Inflate the occupied positions by a given amount inflate(MAP, R) inflates each occupied position of the binary
% occupancy grid by at least R meters.
mapInflated = copy(map);
inflate(mapInflated,robotRadius);

show(mapInflated)

%% Path palnning using PRM ( Proboilistic Roadmap )

% Create a robotics.PRM object and define the associated attributes.
prm = robotics.PRM;

% Assign the inflated map to the PRM object
prm.Map = mapInflated;

% Define the number of PRM nodes to be used during PRM construction. PRM constructs a roadmap using a given number of nodes on the given map. 
prm.NumNodes = 200;

% Define the maximum allowed distance between two connected nodes on the map.
prm.ConnectionDistance = 5;

% Define start and end locations on the map for the path planner to use.
startLocation = [3 5];
endLocation = [26.25 20.22];

% Search for a path between start and end locations using the robotics.PRM.findpath function.
path = findpath(prm, startLocation, endLocation)


%% tunning node numbers

while isempty(path)
    % No feasible path found yet, increase the number of nodes
    prm.NumNodes = prm.NumNodes + 10;

    % Use the |update| function to re-create the PRM roadmap with the changed
    % attribute
    update(prm);

    % Search for a feasible path with the updated PRM
    path = findpath(prm, startLocation, endLocation);
end
% Display path
path
show(prm)




