clearvars; clc; close all;

sublist = {'BOCH02', 'BOCH04', 'BOCH05'};


include = {'mi', 'mi_bhbf'};
excludepat  = {};
depthlevel  = 3;

eventpath = 'analysis/navigation/new/waypoints/';
figpath   = './figures/parkour/new/';
do_save   = true;

% Create/Check for figures
util_mkdir(pwd, figpath);

% Default Waypoints values
StartEvent    = 500;
Waypoints     = [501 502 503 504];
NumWaypoints  = length(Waypoints);
CommandEvents = [26117 26118];
CommandLabels = {'left', 'right'};

NumSubjects = length(sublist);

TYP = []; DUR = []; LBL = []; TIM = []; Rk = []; Sk = [];
for sId = 1:NumSubjects
    csubject = sublist{sId};
    files = util_getfile3(eventpath, '.mat', 'include', [csubject include], 'exclude', excludepat, 'level', depthlevel);
    NumFiles = length(files);

    
    for fId = 1:NumFiles
        util_disp(['[io] - Loading file ' num2str(fId, '%02d') '/' num2str(NumFiles, '%02d') ': ' files{fId}], 'b');
        cevents = load(files{fId});
        
        TYP = cat(1, TYP, cevents.event.TYP);
        DUR = cat(1, DUR, cevents.event.DUR);
        LBL = cat(1, LBL, cevents.event.LBL);
        TIM = cat(1, TIM, cevents.event.TIM);
        Rk  = cat(1, Rk, fId*ones(length(cevents.event.TYP), 1));
        Sk  = cat(1, Sk, sId*ones(length(cevents.event.TYP), 1));
    end
end



Performances = nan(NumWaypoints, NumSubjects);

for sId = 1:NumSubjects
    for wId = 1:NumWaypoints
        cdur = DUR(Sk == sId & TYP == Waypoints(wId));
        Performances(wId, sId) = 1 - sum(isnan(cdur))./length(cdur);
        
    end
end

fig = figure;

bar(Performances);
ylim([0 1.1]);
grid on;

xlabel('waypoint');
ylabel('[%]');
title('Navigation performances');
legend(sublist);

%% Saving figures

if do_save == false
    return
end

figname = fullfile(figpath, 'group_parkour_performances.pdf');


util_disp(['[out] - Saving figure in: ' figname]);
fig_export(fig, figname, '-pdf', 'landscape', '-fillpage');
