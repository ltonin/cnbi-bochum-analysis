function find_nrec_nevt(filesize, headlength, nchannels, samplerate, minnrec)


    % nevt = (filesize - headlength - nrec*nchannels*4*samplerate - 8)/6;

    nevts = [];
    nrecs = [];
    for recId = minnrec:3000
        cnrec = recId;
        cnevt = (filesize - headlength - cnrec*nchannels*4*samplerate - 8)/6;
        
        if(mod(cnevt, 1) == 0)
            nrecs = cat(1, nrecs, cnrec);
            nevts = cat(1, nevts, cnevt);
        end
    end

    keyboard

end