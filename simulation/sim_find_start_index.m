function index = sim_find_start_index(velx, refvel, epsilon)

    if nargin == 2
        epsilon = 0.1;
    end
    
    index = find(abs(velx - refvel) <= epsilon, 1, 'first');
end