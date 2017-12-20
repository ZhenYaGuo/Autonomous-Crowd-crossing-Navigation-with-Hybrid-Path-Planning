function config = loadConfig(config_file)
% load the configuration file
%
%  arguments:
%   config_file     string, which configuration file to load
%


% get the path from the config file -> to read the images
config_path = fileparts(config_file);
if strcmp(config_path, '') == 1
    config_path = '.';
end

fid = fopen(config_file);
input = textscan(fid, '%s=%s');
fclose(fid);

keynames = input{1};
values = input{2};

%convert numerical values from string to double
v = str2double(values);
idx = ~isnan(v);
values(idx) = num2cell(v(idx));

config = cell2struct(values, keynames);

% Read the map image

file = config.map_build;
file_name = [config_path '/' file];
img_build = imread(file_name);

% Decode images
% 1) Wall and Obstacle: Black (0,0,0)
config.img_wall = (img_build(:, :, 1) ==   0 ...
    & img_build(:, :, 2) ==   0 ...
    & img_build(:, :, 3) ==   0);
% 2) Spawn: Magenta (255,0,255)
config.img_spawn = (img_build(:, :, 1) == 255 ...
    & img_build(:, :, 2) ==   0 ...
    & img_build(:, :, 3) == 255);
% 3) Exit: Red (255,0,0)
config.img_exit = (img_build(:, :, 1) ==   255 ...
    & img_build(:, :, 2) == 0 ...
    & img_build(:, :, 3) ==   0);
% 4) Entrance: Blue (0,0,255)
config.img_entrance = (img_build(:, :, 1) == 0 ...
    & img_build(:, :, 2) ==   0 ...
    & img_build(:, :, 3) ==   255);

% Init the plot image here, because this won't change
config.img_plot = 4*config.img_wall ...
    + 3*config.img_entrance ...
    + 2*config.img_exit ...
    + 1*config.img_spawn;
config.color_map = [1 1 1; 0.9 0.9 0.9; 0 1 0; 0.4 0.4 1; 1 0.4 0.4; 0 0 0];

