function nevt = find_nevt_from_nrec(filesize, headlength, nrec, nchannels, samplerate)
    nevt = (filesize - headlength - nrec*nchannels*4*samplerate - 8)/6;
end