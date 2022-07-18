function [distance, index] = sim_find_minimum_distance(x, y, targets, radius)

    if nargin == 3
        radius = 1.0;
    end

    nsamples = length(x);
    if isequal(nsamples, length(y)) == false
        error('chk:length', 'x and y must have the same lenght');
    end

    ncoordinates = size(targets, 1);
    ntargets     = size(targets, 2);

    if isequal(ncoordinates, 2) == false
        error('chk:coor', 'First dimension of positions must have length 2 (x and y)')
    end

    
    mindist_val = inf(ntargets, 1);
    mindist_idx = nan(ntargets, 1);

    currtarget_id = 1;

    for sId = 1:nsamples
        currtarget_x = targets(1, currtarget_id); 
        currtarget_y = targets(2, currtarget_id);

        currpose_x = x(sId);
        currpose_y = y(sId);

        currdistance = hypot((currtarget_x - currpose_x), (currtarget_y - currpose_y));

        if (currdistance < radius)
            
            currmindist_val = mindist_val(currtarget_id);

            if(currdistance < currmindist_val)
                mindist_val(currtarget_id) = currdistance;
                mindist_idx(currtarget_id) = sId;
            elseif (currdistance > currmindist_val)
                currtarget_id = currtarget_id + 1;
            end
        
        end

        if (currtarget_id > ntargets)
            break;
        end

    end

    distance = mindist_val;
    index    = mindist_idx;
end


