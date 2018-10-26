% datapath = '/mnt/data/mi_wheelchair_bochum/BOCH02_mi_wheelchair/';
% file = [datapath '20181001/BOCH02.20181001.144330.online.mi.mi_bhbf.wheelchair.online.gdf'];

datapath = '/mnt/data/mi_wheelchair_bochum/BOCH01_mi_wheelchair/';
file = [datapath '20181018/BOCH01.20181018.103758.online.mi.mi_bhbf.wheelchair.online.gdf'];

[s,h] = sload(file);

filter_band_emg = [1 80];

filter_order = 4;

    
[b,a] = butter(filter_order,filter_band_emg / (h.SampleRate / 2), 'bandpass');

for ch=1:size(s,2)
    s(:,ch) = filtfilt(b,a,s(:,ch));
end



events.position = [];
events.name = {};
for ev=1:length(h.EVENT.TYP)
        events.position(end+1) = h.EVENT.POS(ev);
        events.name{end+1} = num2str(h.EVENT.TYP(ev));
end

eegplot(s', 100, h.SampleRate, 7, events);