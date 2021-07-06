clearvars; clc;

subject = 'BOCH02';

include  = {subject, 'mi', 'mi_bhbf', 'control', 'raw'};
excludepat  = {};
depthlevel  = 3;

experiment  = 'mi_wheelchair';
datapath    = './analysis/navigation/';

savedir     = 'analysis/navigation/new/processed';

files = util_getfile3(datapath, '.mat', 'include', include, 'exclude', excludepat, 'level', depthlevel);
NumFiles = length(files);

% Create/Check for savepath
util_mkdir(pwd, savedir);

% Default values
SmoothOrder      = 1;
SmoothBand       = 0.2;
SmoothFreq       = 1;
MaxEventMismatch = 1; % [s]
MaxCols = 5;

for fId = 1:NumFiles
    [path, name, ext] = fileparts(files{fId});
    
    util_disp(['[io]   + Loading file ' num2str(fId) '/' num2str(NumFiles) ' from ' path ':'], 'b'); 
    data = load(fullfile(path, [name ext]));
    
    % Extract data
    P_raw      = data.pose.xy;
    Q_raw      = data.pose.orientation;
    TP         = data.pose.T;
    Vx_raw     = data.velocity.vx;
    Vz_raw     = data.velocity.vz;
    TV         = data.velocity.T;
    map        = data.map;
    events_raw = data.events;
    
    %% Processing data
    util_disp('[proc] + Processing of navigation data', 'b');
    
    % Interpolate pose
    util_disp('       |- Smoothing pose values');
    S_raw = support_smoothing_filter(P_raw, SmoothOrder, SmoothBand, SmoothFreq);  
    
    % Compute orientation from quaternion
    util_disp('       |- Compute orientation from quaternions');
    [Y_raw, I_raw, R_raw] = quat2angle(Q_raw);
    
    % Unwrap orientation
    util_disp('       |- Unwrap orientation values');
    Y_raw = unwrap(Y_raw);
    I_raw = unwrap(I_raw);
    R_raw = unwrap(R_raw);
    
    % Smoothing orientation
    util_disp('       |- Smoothing orientations');
    sY_raw = support_smoothing_filter(Y_raw, SmoothOrder, SmoothBand, SmoothFreq);
    sI_raw = support_smoothing_filter(I_raw, SmoothOrder, SmoothBand, SmoothFreq);
    sR_raw = support_smoothing_filter(R_raw, SmoothOrder, SmoothBand, SmoothFreq);
    
    % Creating unique temporal support
    util_disp('       |- Creating unique temporal support');
    T = sort([TP; TV]);
    
    % Align pose data to the temporal support
    util_disp('       |- Aligning pose to temporal support');
    aP = support_align_timeseries(T, P_raw, TP);
    aS = support_align_timeseries(T, S_raw, TP);
    
    % Align orientation data to the temporal support
    util_disp('       |- Aligning orientation to temporal support');
    aY  = support_align_timeseries(T, Y_raw, TP);
    aI  = support_align_timeseries(T, I_raw, TP);
    aR  = support_align_timeseries(T, R_raw, TP);
    asY = support_align_timeseries(T, sY_raw, TP);
    asI = support_align_timeseries(T, sI_raw, TP);
    asR = support_align_timeseries(T, sR_raw, TP);
    
    % Align velocity data to the temporal support
    util_disp('       |- Aligning velocity to temporal support');
    aVx = support_align_timeseries(T, Vx_raw, TV);
    aVz = support_align_timeseries(T, Vz_raw, TV);
    
    % Fillment NaNs values
    util_disp('       |- Filling NaNs for pose timeseries');
    P = fillmissing(aP, 'linear');
    S = fillmissing(aS, 'linear');
    
    util_disp('       |- Filling NaNs for orientation timeseries');
    Y  = fillmissing(aY, 'linear');
    I  = fillmissing(aI, 'linear');
    R  = fillmissing(aR, 'linear');
    sY = fillmissing(asY, 'linear');
    sI = fillmissing(asI, 'linear');
    sR = fillmissing(asR, 'linear');
    
    util_disp('       |- Filling NaNs for velocity timeseries');
    Vx = fillmissing(aVx, 'linear');
    Vz = fillmissing(aVz, 'linear');
    
    %% Processing events
    util_disp('[proc] + Processing events', 'b'); 
    util_disp('       |- Align events to temporal support');
    [events, mismatch] = cnbibochum_events_align(T, events_raw, MaxEventMismatch);
    
    %% Creating data vector
    pose.xy        = P;
    spose.xy       = S;
    orientation.x  = R;
    orientation.y  = I;
    orientation.z  = Y;
    sorientation.x = sR;
    sorientation.y = sI;
    sorientation.z = sY;
    velocity.vx    = Vx;
    velocity.vz    = Vz;

    %% Saving data
    sfilename = fullfile(savedir, [name '.mat']);
    util_disp(['[out] - Saving motion data in: ' sfilename], 'b');
    save(sfilename, 'pose', 'spose', 'orientation', 'sorientation', 'velocity', 'map', 'events', 'mismatch', 'T'); 
    
    %% Plotting trajectories
    subplot(ceil(NumFiles/MaxCols), MaxCols, fId);
    cnbibochum_show_map(map.info.x, map.info.y, map.data);
    hold on
    plot(pose.xy(:, 2) - map.info.origin(2), pose.xy(:, 1)- map.info.origin(1), '.r');
    plot(spose.xy(:, 2) - map.info.origin(2), spose.xy(:, 1)- map.info.origin(1), 'g', 'LineWidth', 1);
    hold off;
    title(support_get_datetime(name));
    xlabel('[m]');
    ylabel('[m]');

end
fig_set_position(gcf, 'Top');
sgtitle([subject ' |  trajectories']);


%% ----------------- Functions ------------------- %%
function fdata = support_smoothing_filter(data, order, band, freq)

    fdata = nan(size(data));
    for cId = 1:size(data, 2)
        fdata(:, cId) = filt_highlow(data(:, cId), order, band, freq, 'low');
    end
    
end

function aseries = support_align_timeseries(T, series, Tseries)

    aseries = nan(length(T), size(series, 2));
    idx = [];
    for tId = 1:length(Tseries)
        [~, tidx] = min( abs(T - Tseries(tId)) );
        idx = cat(1, idx, tidx);
    end
    
    for cId = 1:size(series, 2)
        aseries(idx, cId) = series(:, cId);
    end
    
end

function str = support_get_datetime(name)
    token = regexp(name, '\w+.(\d+.\d+).\.*', 'tokens');
    str = datestr(datetime([token{1}], 'InputFormat', 'yyyyMMdd.HHmmss'),  'dd-mmm-yyyy HH:MM:SS');
end
