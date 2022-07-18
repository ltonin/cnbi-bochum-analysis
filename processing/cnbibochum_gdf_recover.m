clearvars;

filetest1 = '/mnt/data/Research/mi_wheelchair_bochum/BOCH04_mi_wheelchair/20190412/BOCH04.20190412.120028.offline.mi.mi_bhbf.wheelchair.offline.gdf';
filetest2 = '/mnt/data/Research/mi_wheelchair_bochum/BOCH02_mi_wheelchair/20190129/BOCH02.20190129.144159.online.mi.mi_bhbf.wheelchair.control.gdf';


filepath{1} = '/mnt/data/Research/mi_wheelchair_bochum/BOCH04_mi_wheelchair/20190612/BOCH04.20190612.135254.online.mi.mi_bhbf.wheelchair.control.gdf';
filepath{2} = '/mnt/data/Research/mi_wheelchair_bochum/BOCH04_mi_wheelchair/20190614/BOCH04.20190614.112227.online.mi.mi_bhbf.wheelchair.control.gdf';
filepath{3} = '/mnt/data/Research/mi_wheelchair_bochum/BOCH04_mi_wheelchair/20190614/BOCH04.20190614.120259.online.mi.mi_bhbf.wheelchair.control.gdf';

filepath{4} = '/mnt/data/Research/mi_wheelchair_bochum/BOCH05_mi_wheelchair/20190529/BOCH05.20190529.122718.online.mi.mi_bhbf.wheelchair.control.gdf';
filepath{5} = '/mnt/data/Research/mi_wheelchair_bochum/BOCH05_mi_wheelchair/20190529/BOCH05.20190529.130840.online.mi.mi_bhbf.wheelchair.control.gdf';
filepath{6} = '/mnt/data/Research/mi_wheelchair_bochum/BOCH05_mi_wheelchair/20190529/BOCH05.20190529.124608.online.mi.mi_bhbf.wheelchair.control.gdf';

% [stest1, htest1] = sload(filetest1);
% [stest2, htest2] = sload(filetest2);


[s, h] = sload(filepath{5});
