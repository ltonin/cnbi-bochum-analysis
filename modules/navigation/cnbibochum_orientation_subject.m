clearvars; 

sublist = {'SIM01', 'BOCH02', 'BOCH04', 'BOCH05'};
NumSubjects = length(sublist);




datapath  = 'analysis/navigation/';
eventpath = 'analysis/navigation/new/waypoints/';
figpath   = './figures/parkour/new/';
do_save   = false;

Smooth = cell(NumSubjects, 1);

for sId = 1:NumSubjects
    subject = sublist{sId};
    include = {subject, 'mi', 'mi_bhbf', 'processed'};
    excludepat  = {};
    depthlevel  = 3;
    files = util_getfile3(datapath, '.mat', 'include', include, 'exclude', excludepat, 'level', depthlevel);
    NumFiles = length(files);

    % Create/Check for figures
    util_mkdir(pwd, figpath);

    % Default Waypoints values
    StartEvent    = 500;
    Waypoints     = [501 502 503 504];
    NumWaypoints  = length(Waypoints);
    CommandEvents = [26117 26118];
    CommandLabels = {'left', 'right'};


    %% Concatenate files
    [nav, map, events, labels] = cnbibochum_navigation_concatenate_data(files, eventpath);

    % Extract data and events
    Wk = proc_get_event3(Waypoints, nav.T, events.POS, events.TYP, events.DUR);
    Tk = proc_get_event3(StartEvent, nav.T, events.POS, events.TYP, events.DUR);
    Ck = proc_get_event3(CommandEvents, nav.T, events.POS, events.TYP, events.DUR);
    P = nav.P;
    O = nav.Ox;
    T = nav.T;
    Vx = nav.Vx;
    Rk = labels.Rk;
    Dk = labels.Dk;
    map = map(1);
    Xk = cnbibochum_get_data_validity(subject, Rk, Wk);
    Nk  = support_get_navigation_labels(events, Rk);
    Ck = support_filter_commands(T, Ck, 1.5);

    Runs = unique(Rk);
    NumRuns = length(Runs);
    Days = unique(Dk);
    NumDays = length(Days);

    % Add offset for simulated runs
    if strcmp(subject, 'SIM01')
        for rId = 1:NumRuns
            cindex = Rk == Runs(rId);
            cp= P(cindex, 1);
            cp(end-1500:end, 1) = cp(end-1500:end, 1) - 0.5;
            P(cindex, 1) = cp;
        end
    end

    SSmooth = nan(NumWaypoints, NumRuns);
    for rId = 1:NumRuns
        for wId = 1:NumWaypoints
            cindex = Rk == Runs(rId) & Wk == Waypoints(wId) & Tk > 0 & Nk == true;

            SSmooth(wId, rId) = smoothness(O(cindex));
        end

    end
    
    Smooth{sId} = SSmooth;
end

%% Figure
fig1 = figure;
fig_set_position(fig1, 'All');

NumRows = 2;
NumCols = 2;
for sId = 1:NumSubjects
    subplot(NumRows, NumCols, sId);
    boxcolors = colororder;
    boxplot((Smooth{sId})', 'labels', [repmat('wp', NumWaypoints, 1) num2str(Waypoints' - StartEvent)] ,'colors', boxcolors(1:NumWaypoints, :))
    grid on;
    xlabel('waypoint');
    % ylabel('[m]');
%     ylim([0 0.5]);
    title([sublist{sId} '| Distribution of smoothness per waypoint']);

end
%% Support functions
function Nk = support_get_navigation_labels(events, Rk)

    nsamples = length(Rk);
    Runs = unique(Rk);
    NumRuns = length(Runs);
    
    Nk = false(nsamples, 1);
    
    for rId = 1:NumRuns
        cstart = find(Rk == Runs(rId), 1, 'first');
        cstop  = find(Rk == Runs(rId), 1, 'last');
        
        cindex = events.POS >= cstart & events.POS <= cstop & (events.TYP == 1 | events.TYP == 2 | events.TYP == 3);
        evtTYP = events.TYP(cindex);
        evtPOS = events.POS(cindex);
        evtLBL = events.LBL(cindex);
        
        evt_pos_nav = evtPOS(evtTYP == 2);
        evt_pos_oth = evtPOS(evtTYP ~= 2);
        
        for nId = 1:length(evt_pos_nav)
            cnav_pos_start = evt_pos_nav(nId);
            cnav_pos_stop_idx  = find(evt_pos_oth > cnav_pos_start, 1, 'first');
            
            if( isempty(cnav_pos_stop_idx) == true)
                cnav_pos_stop = cstop;
            else
                cnav_pos_stop = evt_pos_oth(cnav_pos_stop_idx);
            end
            
            Nk(cnav_pos_start:cnav_pos_stop) = true;
            
        end
    end

end

function nCk = support_filter_commands(T, Ck, timetolerance)

    POS = find(Ck == 26117 | Ck == 26118);
    TYP = Ck(Ck == 26117 | Ck == 26118);
    TIM = T(POS);
    
    MSK = [true; diff(TIM) > timetolerance | diff(TYP) ~=0];

    nCk = Ck;
    nCk(POS(~MSK)) = 0;
    
end

function S = smoothness(x)
    
    % first derivative
    dx = diff(x);
    
    % normalization
    ndx = (dx - mean(dx))./std(dx);
    ndx = dx;
    % second derivative
    d2x = diff(ndx);
    
    % smoothness
    S = sum(d2x.^2)./4;
end

