clearvars; clc;

filename = './analysis/navigation/new/raw/BOCH02.20190128.142219.online.mi.mi_bhbf.wheelchair.control.navigation.mat';


[path, name, ext] = fileparts(filename);
data = load(fullfile(path, [name ext]));
map        = data.map;

%% Plotting trajectories

h = cnbibochum_show_map(map.info.x, map.info.y, map.data, -3.5);
title(support_get_datetime(name));
xlabel('[m]');
ylabel('[m]');
