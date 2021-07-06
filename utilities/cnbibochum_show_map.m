function h = cnbibochum_show_map(x, y, data, rotation)
    if nargin == 3
        rotation = 0;
    end
    
    if abs(rotation) > 0
        data = imrotate(data, rotation);
    end
    h = imagesc(y, x, mat2gray(data)); 
    colormap(flipud(gray));
    set(gca, 'YDir', 'normal', 'XDir', 'reverse'); 
    grid minor;
    axis image;
end
