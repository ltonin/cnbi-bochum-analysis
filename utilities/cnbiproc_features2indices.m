function indices = cnbiproc_features2indices(features, freqgrid)
% indices = cnbiproc_features2indices(features, freqgrid)
%
% The function convert the standard CNBI SMR features structure to indices. 
%
% Input:
%   - features          Standard CNBI SMR features structure with the
%                       following fields:
%                       .channels => vector with channel ids of features
%                       .bands    => cell array with length equal to the
%                                    total number of channels, and for each
%                                    element the frequency (in Hz) of
%                                    feature for that channel
%   - freqgrid          Frequency grid
%
% Output:
%   - indices           Vector with feature indices

    nchans = length(features.bands);
    nfreqs = length(freqgrid);
    
    indices = [];
    

    for chId = 1:length(features.channels)
        cchan = features.channels(chId);

        for fId = 1:length(features.bands{cchan})
            cband = features.bands{cchan}(fId);
            [~, cfreq] = intersect(freqgrid, cband);
            if(isempty(cfreq))
                error('chk:frq', ['Frequency ' num2str(cband) ' not belongs to the frequency grid provided']);
            end
            indices = cat(1, indices, sub2ind([nfreqs nchans], cfreq, cchan));
        end
    end

end