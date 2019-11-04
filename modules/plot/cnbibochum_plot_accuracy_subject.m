clearvars; clc; close all;

subject = 'BOCH04';

spatialfilter = 'laplacian';
artifactrej   = 'none'; % {'FORCe', 'none'}
pattern   = [subject '_probabilities_' spatialfilter '.mat'];
datapath  = ['analysis/' artifactrej '/accuracy/' spatialfilter '/'];


%% Loading fisher score data
util_bdisp(['[io] - Importing probabilities data for subject ' subject ' from ' datapath]);
data = load([datapath pattern]);

Ck      = data.labels.samples.Ck;
Rk      = data.labels.samples.Rk;
Runs    = unique(Rk);
NumRuns = length(Runs); 

pp = data.probabilities.raw;

%% Computing single sample accuracy per run

rejclf = 0.55;
SSAccuracy = nan(NumRuns, 1);
for rId = 1:NumRuns
   cindex = Rk == Runs(rId);
   
   cpp = pp(cindex, 1);
   
   cpp(cpp >= 0.5) = 771;
   cpp(cpp < 0.5) = 773;
   
   idx = Ck(cindex) > 0;
   
   SSAccuracy(rId) = sum(cpp(idx) == Ck(idx))./sum(idx);
    
end

