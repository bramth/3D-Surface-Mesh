function [im_fix] = remove_background(im)
    % Removes background in image, assuming lighting as used in given
    % images.
    
    im_gray = rgb2gray(im);
    im_bw = edge(im_gray,'canny',[0.01,0.4],4);
    
    se = strel('diamond',5);
    im_bw = imdilate(im_bw,se);
    
    
    se = strel('diamond',6);
    im_bw = imclose(im_bw,se);
    
    figure;
    im_bw_fill = imfill(im_bw);
    close;
    
    
    im_fix = im_bw_fill .* im;
    
    im_fix(im_fix == 0) = NaN;
    
    figure;
    imshow(im_fix);
end
