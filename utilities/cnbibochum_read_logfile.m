function log = cnbibochum_read_logfile(logpath, targetname) 


    fid = fopen(logpath);
    
    tline = fgetl(fid);
    classifier = '';
    rejection  = '';
    integration = '';
    thresholds.first = '';
    thresholds.second = '';
    while ischar(tline)
        if(~contains(tline, targetname) == false)
            classifier  = regexp(tline, '(?<=classifier=)\w*.mat', 'match');
            rejection   = regexp(tline, '(?<=rejection=)[-]?+\d+\.\d+', 'match');
            integration = regexp(tline, '(?<=integration=)[-]?+\d+\.\d+', 'match');
            thresholds  = regexp(tline, '(?<=thresholds=)\(\s(?<first>[-]?+\d+\.\d+)\s(?<second>[-]?+\d+\.\d+)\s\)', 'names');
            break
        end
        tline = fgetl(fid);
    end
    
    fclose(fid);
    
    log.classifier  = char(classifier);
    log.rejection   = str2double(rejection);
    log.integration = str2double(integration);
    log.thresholds  = [];
    if isempty(thresholds) == false
        log.thresholds = [str2double(thresholds.first) str2double(thresholds.second)];
    end

end
