function [files, folders] = cnbibochum_util_getdata(datapath, pattern, subfolder)
% [files, folders] = cnbibochum_util_getdata(datapath, pattern [, subfolder])
%
% cnbirob_util_getdata returns all the files in the datapath that match the provided pattern.
% If subfolder argument is provided the function looks for the file in
% datapath/subfolder.
% Input:
%   - datapath  Path where the function starts looking into
%   - pattern   Pattern of the file to look for (wildcard allowed)
%   - subfolder [optional] if provided the function looks into datapath/subfolder
%
% Output:
%   - files     Cell array with the found files
%   - folders   Cell array with the found folders 

    if nargin == 2
        subfolder = '';
    end

%% First level directory access
    % Getting all the files in the given datapath
    entries = dir(datapath);
    folders = {};
    % Storing the id of the files that are not folders or that are '.', '..'
    for eId = 1:length(entries)
        cname   = entries(eId).name;
        cfolder = entries(eId).folder;
        ctype   = entries(eId).isdir;
        if( strcmp(cname, '.') == false && strcmp(cname, '..') == false && ctype == true )
            folders = cat(1, folders, fullfile(cfolder, cname));
        end
    end
    nfolders = length(folders);
 
%% Iterate inside each sub-directory
    files = {};
    for fId = 1:nfolders
       cfolder = fullfile(folders{fId}, subfolder);
       
       % Check if the subfolder exists
       if(exist(cfolder, 'file') == false)
           error('chk:sbf', ['Subfolder ' subfolder ' in ' folders{fId} ' does not exist']);
       end

       centries = dir(fullfile(cfolder, pattern));
       
       for eId = 1:length(centries)
           cname  = centries(eId).name;
           cpath  = centries(eId).folder;
           isfold = centries(eId).isdir;
           
           if (isfold == true)
               continue;
           end
           
           files = cat(1, files, fullfile(cpath, cname));
       end

       if(isempty(files))
           warning('chk:file', ['No files matching ''' pattern ''' found in ' cfolder '/']);
       end 
        
    end
end

