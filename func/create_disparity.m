function [disparity_map] = create_disparity(im1,im2,disparity_range,plotting)

    bs = 15;        %BlockSize              default 15
    cTH = 0.7;      %ContrastThreshold      default 0.5
    uTH = 15;       %UniquenessThreshold    default 15
    dTH = 15;       %DistanceThreshold      default []
    
    im1gr = (rgb2gray(im1));
    im2gr = (rgb2gray(im2));

    disparity_map = disparity(im1gr,im2gr,'DisparityRange',disparity_range,...
        'ContrastThreshold',cTH, 'UniquenessThreshold',uTH, 'DistanceThreshold',dTH,'BlockSize',bs);
    if plotting == true
        figure
        imshow(disparity_map,disparity_range);
        colorbar
    end
end