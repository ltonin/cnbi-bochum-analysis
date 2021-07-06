function Vk = cnbibochum_get_data_validity(subject, Rk, Wk)

    switch(subject)
        case 'BOCH02'
            Vk = BOCH02_get_validity(Rk, Wk);
        case 'BOCH04'
            Vk = BOCH04_get_validity(Rk, Wk);
        case 'BOCH05'
            Vk = BOCH05_get_validity(Rk, Wk);
        case 'SIM01'
            Vk = SIM01_get_validity(Rk, Wk);
        otherwise
            error(['Cannot generate validity vector for subject ' subject ': subject unknown']);
    end
end


function Vk = BOCH02_get_validity(Rk, Wk)

    Vk = true(length(Rk), 1);
    
    % Run 2
    Vk(Rk == 2 & Wk == 504) = false;
    
    % Run 3
    Vk(Rk == 3 & Wk == 504) = false;

end

function Vk = BOCH04_get_validity(Rk, Wk)


    Vk = true(length(Rk), 1);
    
    % Run 1
    Vk(Rk == 1 & Wk == 502) = false;
    Vk(Rk == 1 & Wk == 504) = false;
    
    % Run 2
    Vk(Rk == 2 & Wk == 502) = false;
    Vk(Rk == 2 & Wk == 503) = false;
    Vk(Rk == 2 & Wk == 504) = false;
    
    % Run 3
    Vk(Rk == 3 & Wk == 502) = false;
    Vk(Rk == 3 & Wk == 503) = false;
    Vk(Rk == 3 & Wk == 504) = false;
    
    % Run 4
    Vk(Rk == 4 & Wk == 502) = false;
    Vk(Rk == 4 & Wk == 504) = false;
    
    % Run 5
    Vk(Rk == 5 & Wk == 504) = false;

end

function Vk = BOCH05_get_validity(Rk, Wk)

    Vk = true(length(Rk), 1);
    
    % Run 1
    Vk(Rk == 1 & Wk == 504) = false;
    
    % Run 2
    Vk(Rk == 2 & Wk == 502) = false;
    Vk(Rk == 2 & Wk == 503) = false;
    Vk(Rk == 2 & Wk == 504) = false;
    
    % Run 3
    Vk(Rk == 3 & Wk == 502) = false;
    Vk(Rk == 3 & Wk == 503) = false;
    Vk(Rk == 3 & Wk == 504) = false;
    
    % Run 4
    Vk(Rk == 4 & Wk == 504) = false;
    
    % Run 6
    Vk(Rk == 6 & Wk == 504) = false;

end

function Vk = SIM01_get_validity(Rk, Wk)

    Vk = true(length(Rk), 1);
end


