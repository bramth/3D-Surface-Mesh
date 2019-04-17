clear all,  clc, format compact, close all
warning('off','images:initSize:adjustingMag')

%% Open images 

% im{1} = imread('subject4\subject4_Left\subject4_Left_1.jpg');
% im{2} = imread('subject4\subject4_Middle\subject4_Middle_1.jpg');
% im{3} = imread('subject4\subject4_Right\subject4_Right_1.jpg');
% 
% for i = 1:length(im)
%     im{i} = im2double(im{i});
% end

%% determine stereoparameters

% stereoParams{1}=Cam_calib2_lm;
% stereoParams{2}=Cam_calib2_mr;

% save stereoParam stereoParams

load stereoParam

%% Histogram matching

% im{1} = imhistmatch(im{1},im{2});
% im{3} = imhistmatch(im{3},im{2});

%% Remove background
 
% for ii = 1:length(im)
%     im{ii} = remove_background(im{ii});
% end
% 
% save im_nobg4 im

load im_nobg4

%% Stereo rectification

[im_lm{2},im_lm{1}] = rectifyStereoImages(im{2},im{1},stereoParams{1},'OutputView','full');
[im_mr{1},im_mr{2}] = rectifyStereoImages(im{2},im{3},stereoParams{2},'OutputView','full');

% figure;
% subplot(1,2,1);
% imshow(stereoAnaglyph(im_lm{1},im_lm{2}));
% subplot(1,2,2);
% imshow(stereoAnaglyph(im_mr{1},im_mr{2}));


%% Filter image

% h = fspecial('gaussian',5,1);
% im_lm{1} = imfilter(im_lm{1},h);
% im_lm{2} = imfilter(im_lm{2},h);
% im_mr{1} = imfilter(im_mr{1},h);
% im_mr{2} = imfilter(im_mr{2},h);

%% Disparity map

disp_range{1} = 16*[-28,-15]; 
%disp_range{1} = 16*[-35,-10];
disparity_map{1} = create_disparity(im_lm{2},im_lm{1},disp_range{1},true);

disp_range{2} = 16*[15,34];
%disp_range{2} = 16*[10,35];
disparity_map{2} = create_disparity(im_mr{1},im_mr{2},disp_range{2},true);

%% Obtain unreliables
unreliable{1} = (disparity_map{1}==-realmax('single')) | (1-(rgb2gray(im_lm{2})>0));
unreliable{2} = (disparity_map{2}==-realmax('single')) | (1-(rgb2gray(im_mr{1})>0));

%% Polish disparity map

% ... by filter map
disparity_map_pol{1} = medfilt2(disparity_map{1});
disparity_map_pol{2} = medfilt2(disparity_map{2});


% ... by getting rid of unreliables
for i = 1:length(disparity_map)
    disparity_map_pol{i} = disparity_map{i}.*(1-unreliable{i}); 
end

% ... by interpolating missing values
disparity_map_pol{1}((disparity_map_pol{1}==0) & ((rgb2gray(im_lm{2}))>0)) = NaN;
disparity_map_pol{2}((disparity_map_pol{2}==0) & ((rgb2gray(im_mr{1}))>0)) = NaN;

disparity_map_pol{1}=fillmissing(disparity_map_pol{1},'nearest');
disparity_map_pol{2}=fillmissing(disparity_map_pol{2},'nearest');

%% Reconstruct face
for i = 1:length(disparity_map)
    [point_cloud{i},xyzPoints{i},point_cloud_down{i}] = create_point_cloud(disparity_map_pol{i},stereoParams{i},true);
end

%% Combine both point-clouds

[point_cloud_merge,pc_rms_error] = merge_point_cloud(point_cloud,point_cloud_down,stereoParams,true);


%% Visualise stereographs
create_mesh(disparity_map_pol{1},xyzPoints{1},im_lm{2},unreliable{1})
create_mesh(disparity_map_pol{2},xyzPoints{2},im_mr{1},unreliable{2})
