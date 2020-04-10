function hitmap = cnbibochum_traj2map(T, fieldsize, resolution)
% hitmap = cnbibochum_traj2map(trajectory, fieldsize, resolution)
%
% cnbirob_traj2map creates a hit-map from a given trajectory T. T must be a
% matrix with size [N x 2], where N is number of trajectory coordinates (X, Y).
% fieldsize corresponds to the size of the field where the trajectory has been
% recorded (same units as the provided trajectory T). 
% resolution is the desired resolution of the computed hit-map.

    if size(T, 2) ~= 2
        error('chk:in', 'trajectory must be a matrix with 2 columns')
    end
    
    % Compute the size of the hit-map
    mapsize = ceil(fieldsize/resolution);
    
    % Convert the trajectory to the new resolution
    rT = ceil(T/resolution);
    
    % Ensure that the new trajectory are not outside the map
    rT(rT(:, 1) > mapsize(1), 1) = nan;
    rT(rT(:, 2) > mapsize(2), 2) = nan;
    
    % Instantiate the hit-map matrix
    hitmap = zeros(mapsize);

    % Convert subindex in indexes
    ind = sub2ind(mapsize, rT(:, 1), rT(:, 2));
    
    % Exclude the nan indices
    ind = ind(isnan(ind) == false);
    
    % Store hit in the given indices
    hitmap(ind) = 1;
   
end
