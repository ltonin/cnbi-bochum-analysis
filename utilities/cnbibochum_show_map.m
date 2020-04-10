function cnbibochum_show_map(x, y, data)
    imagesc(y, x, mat2gray(data)); 
    colormap(flipud(gray));
    set(gca, 'YDir', 'normal', 'XDir', 'reverse'); 
    grid minor;
    axis image;
end
