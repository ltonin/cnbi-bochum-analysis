clearvars; close all;

sublist = {'BOCH02', 'BOCH04', 'BOCH05'};
manualsubject  = 'SIM01';

include = {'mi', 'mi_bhbf', 'processed'};
excludepat  = {};
depthlevel  = 3;

datapath  = 'analysis/navigation/';
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

%% Manual data
files_manual  = util_getfile3(datapath, '.mat', 'include', [include manualsubject],  'exclude', excludepat, 'level', depthlevel);
[nav_m,   ~,  events_m, labels_m] = cnbibochum_navigation_concatenate_data(files_manual,  eventpath);

Wk_m = proc_get_event3(Waypoints,     nav_m.T, events_m.POS, events_m.TYP, events_m.DUR);
Tk_m = proc_get_event3(StartEvent,    nav_m.T, events_m.POS, events_m.TYP, events_m.DUR);
Ck_m = proc_get_event3(CommandEvents, nav_m.T, events_m.POS, events_m.TYP, events_m.DUR);
P_m  = nav_m.P;
Vx_m = nav_m.Vx;
T_m  = nav_m.T;
Rk_m = labels_m.Rk;
Xk_m = cnbibochum_get_data_validity(manualsubject, Rk_m, Wk_m);

Runs_m = unique(Rk_m);
NumRuns_m = length(Runs_m);

% Add offset for simulated runs
for rId = 1:NumRuns_m
    cindex = Rk_m == Runs_m(rId);
    cp= P_m(cindex, :);
%     cp(end-1500:end, 1) = cp(end-1500:end, 1) - 0.5;
    cp(:, 1) = cp(:, 1) - 0.7;
    cp(:, 2) = cp(:, 2) + 0.4;
    P_m(cindex, :) = cp;
end

%% Subject data
Wk = []; Tk = []; P = []; T = []; Vx = []; Rk = []; Xk = []; Nk = []; Sk = []; Ck = [];
for sId = 1:NumSubjects
    csubject = sublist{sId};
    files = util_getfile3(datapath, '.mat', 'include', [csubject include], 'exclude', excludepat, 'level', depthlevel);
    NumFiles = length(files);

    %% Concatenate files
    [nav, map, events, labels] = cnbibochum_navigation_concatenate_data(files, eventpath);

    % Extract data and events
    cWk  = proc_get_event3(Waypoints,  nav.T, events.POS, events.TYP, events.DUR);
    cTk  = proc_get_event3(StartEvent, nav.T, events.POS, events.TYP, events.DUR);
    cCk  = proc_get_event3(CommandEvents, nav.T, events.POS, events.TYP, events.DUR);
    cP   = nav.P;
    cT   = nav.T;
    cVx  = nav.Vx;
    cRk  = labels.Rk;
    cXk  = cnbibochum_get_data_validity(csubject, cRk, cWk);
    cNk  = support_get_navigation_labels(events, cRk);
    cCk = support_filter_commands(cT, cCk, 1.5);
    map = map(1);
    
    % Concatenate data
    Wk = cat(1, Wk, cWk);
    Tk = cat(1, Tk, cTk);
    P  = cat(1, P, cP);
    T  = cat(1, T, cT);
    Vx = cat(1, Vx, cVx);
    Rk = cat(1, Rk, cRk);
    Xk = cat(1, Xk, cXk);
    Nk = cat(1, Nk, cNk);
    Ck = cat(1, Ck, cCk);
    Sk = cat(1, Sk, sId*ones(length(cWk), 1));
 
end

Subjects = unique(Sk);
NumSubjects = length(Subjects);

rRk = [];
rSk = [];
for sId = 1:NumSubjects
    cruns = unique(Rk(Sk == Subjects(sId)));
    cnruns = length(cruns);
    rRk = cat(1, rRk, cruns);
    rSk = cat(1, rSk, sId*ones(length(cruns), 1));
end

%% Compute waypoint commands
Commands = [];
for sId = 1:NumSubjects
    cindex = Sk == Subjects(sId);
    Commands = cat(3, Commands, support_waypoint_commands(Ck(cindex), Wk(cindex), Rk(cindex), Xk(cindex)));
end
Commands_m = support_waypoint_commands(Ck_m, Wk_m, Rk_m, Xk_m);


%% Compute reference trajectory (manual)
Tj_m    = reshape_full_trajectories(P_m, Rk_m, Tk_m);
Tj_ref  = nanmean(Tj_m, 3);
Twj_m   = reshape_wp_trajectories(P_m, Wk_m, Rk_m, Tk_m);
Twj_ref = cellfun(@(m) mean(m, 3), Twj_m, 'UniformOutput', false);

%% Compute Frechet distance for each waypoint

WpFrechet = [];
for sId = 1:NumSubjects
    cruns = unique(Rk(Sk == Subjects(sId)));
    cnruns = length(cruns);
    cWpFrechet = nan(NumWaypoints, cnruns);
    for wId = 1:NumWaypoints
        for rId = 1:cnruns
            cindex = Rk == cruns(rId) & Tk > 0 & Wk == Waypoints(wId) & Xk == true & Nk == true & Sk == Subjects(sId);
            if sum(cindex) <= 1
                continue;
            end
            cpath = P(cindex, :);
            rpath = Twj_ref{wId};
            cWpFrechet(wId, rId) = proc_frechet_distance(cpath, rpath);

        end
    end
    
    WpFrechet = cat(2, WpFrechet, cWpFrechet);
end


%% Compute balance commands per subject per run
CommandPerRun_m = squeeze(sum(Commands_m, 2));
BalancePerRun_m = CommandPerRun_m(1, :)./sum(CommandPerRun_m);
Balance_m = mean(BalancePerRun_m);

BalancePerRun = cell(NumSubjects, 1);
BalanceRatio = cell(NumSubjects, 1);
BalanceDistance = cell(NumSubjects, 1);
BalanceDiff = cell(NumSubjects, 1);
for sId = 1:NumSubjects
    ccomm  = squeeze(nansum(Commands(:, :, rSk == Subjects(sId)), 2));
    cbalan = ccomm(1, :)./sum(ccomm);
    
    BalancePerRun{sId} = cbalan;
    
    BalanceRatio{sId} = cbalan./Balance_m;
    BalanceDistance{sId} = hypot(cbalan, Balance_m);
    BalanceDiff{sId} = abs(cbalan - Balance_m);
end

%% Compute balance commands per subject, waypoint and run
BalancePerWp_m = squeeze(Commands_m(1, :, :)./sum(Commands_m));
BalanceWp_m = mean(BalancePerWp_m, 2);

BalancePerWp = cell(NumSubjects, 1);
BalanceWpRatio = cell(NumSubjects, 1);
BalanceDiffWp = cell(NumSubjects, 1);
for sId = 1:NumSubjects
   ccomm = Commands(:, :, rSk == Subjects(sId));
   cbalan = squeeze(ccomm(1, :, :)./sum(ccomm));
   BalancePerWp{sId} = cbalan;
   BalanceWpRatio{sId} = cbalan./BalanceWp_m;
   BalanceDiffWp{sId} = abs(cbalan - Balance_m);
end


%% Figure

fig1 = figure;
fig_set_position(fig1, 'Top');
subplot(1, 3, 1);
x = [];
y = [];
colors = get(gca, 'ColorOrder');
hold on;
h = zeros(NumSubjects, 1);
SumEigen = nan(NumSubjects, 3);
ProdEigen = nan(NumSubjects, 3);
for sId = 1:NumSubjects
    
    sBC = BalanceRatio{sId};
    sWF = nanmean(WpFrechet(:, rSk == Subjects(sId)));
    
    rBC = reshape(sBC, numel(sBC), 1);
    rWF = reshape(sWF, numel(sWF), 1);
    hold on;
    h(sId) = plot(rBC, rWF, 'o', 'Color', colors(sId, :));
    %plot_confidence_elipse(rBC, rWF, 'Color', colors(sId, :));
    e = error_ellipse(cov([rBC rWF]), mean([rBC rWF]));
    SumEigen(sId, 1) = trace(cov([rBC rWF]));
    ProdEigen(sId, 1) = det(cov([rBC rWF]));
    set(e, 'Color', colors(sId, :));
    hold off;
    x = cat(1, x, rBC);
    y = cat(1, y, rWF);
end
[curve,gof] = fit(x,y,'poly2');

hold on;
plot(curve, 'k');
hold off;
ylim([0 3.5]);
plot_vline(1, 'k--');
legend(h, {'S1', 'S2', 'S3'});

hold off;
grid on;
% title('\bf Frechet vs. $\frac{Comm_{BCI}}{Comm_{MAN}}$ (per run)', 'FontSize', 14, 'interpreter', 'latex');
% ylabel('Frechet [m]', 'FontSize', 16, 'interpreter', 'latex');
% xlabel('$\frac{Comm_{BCI}}{Comm_{MAN}}$', 'FontSize', 16, 'interpreter', 'latex');
title('Frechet vs. CommBCI/CommMAN (per run)');
ylabel('Frechet [m]');
xlabel('CommBCI/CommMAN');



hold off;
    
subplot(1, 3, 2);

x = [];
y = [];
colors = get(gca, 'ColorOrder');
hold on;
h = zeros(NumSubjects, 1);
for sId = 1:NumSubjects
    
    sBC = BalanceDiff{sId};
    sWF = nanmean(WpFrechet(:, rSk == Subjects(sId)));
    
    rBC = reshape(sBC, numel(sBC), 1);
    rWF = reshape(sWF, numel(sWF), 1);
    hold on;
    h(sId) = plot(rBC, rWF, 'o', 'Color', colors(sId, :));
    %plot_confidence_elipse(rBC, rWF, 'Color', colors(sId, :));
    e = error_ellipse(cov([rBC rWF]), mean([rBC rWF]));
    SumEigen(sId, 2) = trace(cov([rBC rWF]));
    ProdEigen(sId, 2) = det(cov([rBC rWF]));
    set(e, 'Color', colors(sId, :));
    hold off;
    x = cat(1, x, rBC);
    y = cat(1, y, rWF);
end
[curve,gof3] = fit(x,y,'poly1');

hold on;
plot(curve, 'k');
hold off;

legend(h, {'S1', 'S2', 'S3'});

[rho2, pval2] = corr(x, y, 'rows', 'pairwise');
text(0.35, 2.9, ['rho = ' num2str(rho2, '%4.2f')]);
text(0.35, 2.8, ['p-val = ' num2str(pval2, '%4.2f')]);
ylim([0 3.5]);

hold off;
grid on;
% title('\bf Frechet vs. $abs(Comm_{BCI} - Comm_{MAN})$ (per run)', 'FontSize', 14, 'interpreter', 'latex');
% ylabel('Frechet [m]', 'FontSize', 16, 'interpreter', 'latex');
% xlabel('$abs(Comm_{BCI} - Comm_{MAN})$', 'FontSize', 16, 'interpreter', 'latex');
title('Frechet vs. abs(CommBCI - CommMAN) (per run)');
ylabel('Frechet [m]');
xlabel('abs(CommBCI - CommMAN)');


subplot(1, 3, 3);

x = [];
y = [];
colors = get(gca, 'ColorOrder');
hold on;
h = zeros(NumSubjects, 1);
wSk = [];
for sId = 1:NumSubjects
    
    sBC = BalanceDiffWp{sId};
    sWF = WpFrechet(:, rSk == Subjects(sId));
    
    rBC = reshape(sBC, numel(sBC), 1);
    rWF = reshape(sWF, numel(sWF), 1);
    
    index = ~(isinf(rBC) | isnan(rBC));
    rBC = rBC(index);
    rWF = rWF(index);
    
    hold on;
    h(sId) = plot(rBC, rWF, 'o', 'Color', colors(sId, :));

    e = error_ellipse(nancov([rBC rWF]), nanmean([rBC rWF]));
    set(e, 'Color', colors(sId, :));
    SumEigen(sId, 3) = trace(cov([rBC rWF]));
    ProdEigen(sId, 3) = det(cov([rBC rWF]));
    wSk = cat(1, wSk, sId*ones(length(rBC), 1));
    x = cat(1, x, rBC);
    y = cat(1, y, rWF);
end
[curve,gof4] = fit(x,y,'poly1');

hold on;
plot(curve, 'k');
hold off;
legend(h, {'S1', 'S2', 'S3'});
[rho3, pval3] = corr(x, y, 'rows', 'pairwise');
text(0.35, 2.9, ['rho = ' num2str(rho3, '%4.3f')]);
text(0.35, 2.8, ['p-val = ' num2str(pval3, '%4.3f')]);
ylim([0 3.5]);

hold off;
grid on;
% title('\bf Frechet vs. $abs(Comm_{BCI} - Comm_{MAN})$ (per wp)', 'FontSize', 14, 'interpreter', 'latex');
% ylabel('Frechet [m]', 'FontSize', 16, 'interpreter', 'latex');
% xlabel('$abs(Comm_{BCI} - Comm_{MAN})$', 'FontSize', 16, 'interpreter', 'latex');
title('Frechet vs. CommBCI - CommMAN (per wp)');
ylabel('Frechet [m]');
xlabel('abs(CommBCI - CommMAN)');
    

fig2 = figure;
x = [];
y = [];
colors = get(gca, 'ColorOrder');
hold on;
h = zeros(NumSubjects, 1);
wSk = [];
for sId = 1:NumSubjects
    
    sBC = BalanceDiffWp{sId};
    sWF = WpFrechet(:, rSk == Subjects(sId));
    
    rBC = reshape(sBC, numel(sBC), 1);
    rWF = reshape(sWF, numel(sWF), 1);
    
    index = ~(isinf(rBC) | isnan(rBC));
    rBC = rBC(index);
    rWF = rWF(index);
    
    hold on;
    h(sId) = plot(rBC, rWF, 'o', 'Color', colors(sId, :));

%     e = error_ellipse(nancov([rBC rWF]), nanmean([rBC rWF]));
%     set(e, 'Color', colors(sId, :));
%     SumEigen(sId, 3) = trace(cov([rBC rWF]));
%     ProdEigen(sId, 3) = det(cov([rBC rWF]));
%     wSk = cat(1, wSk, sId*ones(length(rBC), 1));
    x = cat(1, x, rBC);
    y = cat(1, y, rWF);
end
[curve,gof4] = fit(x,y,'poly1');

hold on;
plot(curve, 'k');
hold off;
legend(h, {'S1', 'S2', 'S3'});
[rho3, pval3] = corr(x, y, 'rows', 'pairwise');
text(0.35, 2.9, ['rho = ' num2str(rho3, '%4.3f')]);
text(0.35, 2.8, ['p-val = ' num2str(pval3, '%4.3f')]);
ylim([0 3.5]);

hold off;
grid on;

title('Frechet vs. CommBCI - CommMAN (per wp)');
ylabel('Frechet [m]');
xlabel('abs(CommBCI - CommMAN)');

%% Balance command
fig3 = figure;
cbalance = [];
cgroup   = [];
for sId = 1:NumSubjects
    cbalance = cat(1, cbalance, BalancePerRun{sId}');
    cgroup   = cat(1, cgroup, sId*ones(length(BalancePerRun{sId}), 1));
end
cbalance = cat(1, cbalance, BalancePerRun_m');
cgroup   = cat(1, cgroup, (NumSubjects+1)*ones(length(BalancePerRun_m), 1));

boxplot(cbalance, cgroup, 'labels', {'S1', 'S2', 'S3', 'M'});
ylim([-0.1 1]);
plot_hline(median(BalancePerRun_m), '--k');
plot_vline(NumSubjects+0.5, 'k');
grid on;
xlabel('subject');
ylabel('command ratio');
title('Left/Right command ration');


%% Saving
if do_save == false
    return
end

figname1 = fullfile(figpath, 'group_parkour_balance_correlation_elipses.pdf');
figname2 = fullfile(figpath, 'group_parkour_balance_correlation.pdf');
figname3 = fullfile(figpath, 'group_parkour_balance_left_right.pdf');

util_disp(['[out] - Saving figure in: ' figpath]);
fig_export(fig1, figname1, '-pdf', 'landscape', '-bestfit');
fig_export(fig2, figname2, '-pdf', 'landscape', '-bestfit');
fig_export(fig3, figname3, '-pdf', 'landscape', '-bestfit');

%% Functions

function nCk = support_filter_commands(T, Ck, timetolerance)

    POS = find(Ck == 26117 | Ck == 26118);
    TYP = Ck(Ck == 26117 | Ck == 26118);
    TIM = T(POS);
    
    MSK = [true; diff(TIM) > timetolerance | diff(TYP) ~=0];

    nCk = Ck;
    nCk(POS(~MSK)) = 0;
    
end

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

function C = support_waypoint_commands(Ck, Wk, Rk, Xk)
    Runs = unique(Rk);
    NumRuns = length(Runs);
    Commands = setdiff(unique(Ck), 0);
    NumCommands = length(Commands);
    Waypoints    = setdiff(unique(Wk), 0);
    NumWaypoints = length(Waypoints);

    C = nan(NumCommands, NumWaypoints, NumRuns);
    for rId = 1:NumRuns
        for wId = 1:NumWaypoints
            cindex = Rk == Runs(rId) & Wk == Waypoints(wId) & Xk == true;
            if sum(cindex) <= 1
                continue;
            end
            for cId = 1:NumCommands
               C(cId, wId, rId) = sum(cindex & Ck == Commands(cId));
            end
        end
    end
end

function Tj = reshape_full_trajectories(P, Rk, Tk)

    Runs    = unique(Rk);
    NumRuns = length(Runs);
    
    % Find max length
    maxlen = 0;
    for rId = 1:NumRuns
        cindex = Rk == Runs(rId) & Tk > 0;
        maxlen = max(length(Rk(cindex)), maxlen);
    end

	Tj = nan(maxlen, 2, NumRuns);
    for rId = 1:NumRuns
        cindex = Rk == Runs(rId) & Tk > 0;
        clength = sum(cindex);
        ctraj = P(cindex, :);
        Tj(:, :, rId) =  interp1(1:clength, ctraj, linspace(1, clength, maxlen));
    end
end


function Twj = reshape_wp_trajectories(P, Wk, Rk, Tk)
    
    Waypoints    = setdiff(unique(Wk), 0);
    NumWayPoints = length(Waypoints);
    Runs         = unique(Rk);
    NumRuns      = length(Runs);
    
    Twj = cell(NumWayPoints, 1);
    for wId = 1:NumWayPoints
        % Find max length
        maxlen = 0;
        for rId = 1:NumRuns
            cindex = Rk == Runs(rId) & Tk > 0 & Wk == Waypoints(wId);
            maxlen = max(length(Rk(cindex)), maxlen);
        end

        Tj = nan(maxlen, 2, NumRuns);
        for rId = 1:NumRuns
            cindex = Rk == Runs(rId) & Tk > 0 & Wk == Waypoints(wId);
            clength = sum(cindex);
            ctraj = P(cindex, :);
            Tj(:, :, rId) =  interp1(1:clength, ctraj, linspace(1, clength, maxlen));
        end
        
        Twj{wId} = Tj;
    end
    
end

