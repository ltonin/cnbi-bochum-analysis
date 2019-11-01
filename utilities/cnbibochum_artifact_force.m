function s = cnbibochum_artifact_force(s, winlength, samplerate, chanlocs)

    if ischar(chanlocs)
        tmp = load(chanlocs);
        locs = tmp.chanlocs;
    end
    
    if isstruct(locs) == false
        error('Wrong format channel location file');
    end
        
            
    winSamples = winlength*samplerate;
    winNum     = size(s,1)-winSamples+1;
    
    for winPos = 1:winSamples:winNum
        util_disp_progress(winPos, winNum, '        ');
        s(winPos:winPos+winSamples-1,:) = FORCe( s(winPos:winPos+winSamples-1,:)', samplerate, locs, 0 )';
    end

end