function [waypoints, days] = cnbibochum_get_parkour_data(subject)
    
    switch(subject)
        case 'BOCH02'
            [waypoints, days] = BOCH02_parkour_data;
        case 'BOCH04'
            [waypoints, days] = BOCH04_parkour_data;
        case 'BOCH05'
            [waypoints, days] = BOCH05_parkour_data;
        otherwise
            error(['Parkour data for subject ' subject ' not available']);
    end
    
    

end

function [waypoints, days] = BOCH02_parkour_data

    waypoints(:, 1)  = [1 1 1 1];
    waypoints(:, 2)  = [1 1 1 0];
    waypoints(:, 3)  = [1 1 1 0];
    waypoints(:, 4)  = [1 1 1 1];
    waypoints(:, 5)  = [1 1 1 1];
    waypoints(:, 6)  = [1 1 1 1];
    waypoints(:, 7)  = [1 1 1 1];
    waypoints(:, 8)  = [1 1 1 1];
    waypoints(:, 9)  = [1 1 1 1];
    waypoints(:, 10) = [1 1 1 1];
    
    days.index = [1 1 1 1 1 1 2 2 2 2];
    days.label = {'20190128', '20190129'};

end

function [waypoints, days] = BOCH04_parkour_data

    waypoints(:, 1)  = [1 1 1 0];
    waypoints(:, 2)  = [1 0 1 1];
    waypoints(:, 3)  = [1 1 0 0];
    waypoints(:, 4)  = [1 1 1 1];
    
    days.index = [1 1 2 2];
    days.label = {'20190612', '20190614'};

end

function [waypoints, days] = BOCH05_parkour_data

    waypoints(:, 1)  = [1 1 0 0];
    waypoints(:, 2)  = [1 1 1 0];
    waypoints(:, 3)  = [1 1 1 1];
    waypoints(:, 4)  = [1 1 1 1];
    
    days.index = [1 1 1 1];
    days.label = {'20190529'};

end