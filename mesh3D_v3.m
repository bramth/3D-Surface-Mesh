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
disp_range = 16*[-35,-10];
disparity_map{1} = create_disparity(im_lm{2},im_lm{1},disp_range,true);

disp_range = 16*[10,35];
disparity_map{2} = create_disparity(im_mr{1},im_mr{2},disp_range,true);

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
    [point_cloud{i},point_cloud_down{i}] = create_point_cloud(disparity_map_pol{i},stereoParams{i},true);
    %point_cloud{i} = pcdenoise(point_cloud{i});
end

%% Combine both point-clouds

[tform,~,pc_rms_error] = pcregistericp(point_cloud_down{2},point_cloud_down{1},'Verbose',true);
point_cloud_merge = pcmerge(point_cloud{1},pctransform(point_cloud{2},tform),0.1);

figure
pcshow(point_cloud_merge)
%view([180,90])
title('pc merged')

%% Vis
mesh3d(disparity_map_pol{2},point_cloud{2}.Location,im_lm{2},unreliable{1})



%% Obtain only location
 
for i = 1:length(point_cloud)
    pc_loc{i} = point_cloud{i}.Location;
end

%pc_merge_loc = point_cloud_merge.Location;

%%%% FROM HERE COPY PASTE %%%%

% 
% %% create a connectivity structure
% [M, N] = size(disparity_map_pol{2}); % get image size
% res = 2; % resolution of mesh
% [nI,mI] = meshgrid(1:res:N,1:res:M); % create a 2D meshgrid of pixels, thus defining a resolution grid
% TRI = delaunay(nI(:),mI(:)); % create a triangle connectivity list
% indI = sub2ind([M,N],mI(:),nI(:)); % cast grid points to linear indices
% 
% %% linearize the arrays and adapt to chosen resolution
% 
% %pcl = reshape(pc_merge_loc,N*M,3); % reshape to (N*M)x3
% pcl = reshape(pc_loc{2},N*M,3); % reshape to (N*M)x3
% im_ml_vect = reshape(im_mr{1},[N*M,3]); % reshape to (N*M)x3
% pcl = pcl(indI,:); % select 3D points that are on resolution grid
% im_ml_vect = im_ml_vect(indI,:); % select pixels that are on the resolution grid
% 
% %% remove the unreliable points and the associated triangles
% 
% ind_unreliable = find(unreliable{2}(indI));% get the linear indices of unreliable 3D points
% imem = ismember(TRI(:),ind_unreliable); % find indices of references to unreliable points
% [ir,~] = ind2sub(size(TRI),find(imem)); % get the indices of rows with refs to unreliable points.
% TRI(ir,:) = []; % dispose them
% iused = unique(TRI(:)); % find the ind's of vertices that are in use
% used = zeros(length(pcl),1); % pre-allocate
% used(iused) = 1; % create a map of used vertices
% map2used = cumsum(used); % conversion table from indices of old vertices to the new one
% pcl = pcl(iused,:); % remove the unused vertices
% im_ml_vect = im_ml_vect(iused,:);
% TRI = map2used(TRI); % update the ind's of vertices
% 
% %% create the 3D mesh
% 
% TR = triangulation(TRI,double(pcl)); % create the object
% 
% %% visualize
% figure(5), clf(5), hold on
% TM = trimesh(TR); % plot the mesh
% set(TM,'FaceVertexCData',im_ml_vect); % set colors to input image
% set(TM,'Facecolor','interp');
% % set(TM,'FaceColor','red'); % if you want a colored surface
% set(TM,'EdgeColor','none'); % suppress the edges
% xlabel('x (mm)')
% ylabel('y (mm)')
% zlabel('z (mm)')
% axis([-250 250 -250 250 400 900])
% set(gca,'xdir','reverse')
% set(gca,'zdir','reverse')
% daspect([1,1,1])
% axis tight
% %view([0,90]);


%% Functions

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

function [point_cloud,point_cloud_down] = create_point_cloud(disparity_map,stereoParams,plotting)
    xyzPoints = reconstructScene(disparity_map,stereoParams);
    point_cloud = pointCloud(xyzPoints);
    point_cloud = removeInvalidPoints(point_cloud);
    point_cloud = pcdenoise(point_cloud,'Threshold',0.1);
    point_cloud_down = pcdownsample(point_cloud,'gridAverage',10);

    if plotting == true
        figure;
        pcshow(xyzPoints);
    end
end

function [] = mesh3d(disparityMap,pc,J1,unreliable)
    % Matlab code for creating a 3D surface mesh
    
    % create a connectivity structure
    [M, N] = size(disparityMap); % get image size
    res = 2; % resolution of mesh
    [nI,mI] = meshgrid(1:res:N,1:res:M); % create a 2D meshgrid of pixels, thus defining a resolution grid
    TRI = delaunay(nI(:),mI(:)); % create a triangle connectivity list
    indI = sub2ind([M,N],mI(:),nI(:)); % cast grid points to linear indices
    
    % linearize the arrays and adapt to chosen resolution
    %pcl = reshape(pc,N*M,3); % reshape to (N*M)x3
    pcl = pc;
    J1l = reshape(J1,N*M,3); % reshape to (N*M)x3
    pcl = pcl(indI,:); % select 3D points that are on resolution grid
    J1l = J1l(indI,:); % select pixels that are on the resolution grid
    
    % remove the unreliable points and the associated triangles
    ind_unreliable = find(unreliable(indI));% get the linear indices of unreliable 3D points
    imem = ismember(TRI(:),ind_unreliable); % find indices of references to unreliable points
    [ir,~] = ind2sub(size(TRI),find(imem)); % get the indices of rows with refs to unreliable points.
    TRI(ir,:) = []; % dispose them
    iused = unique(TRI(:)); % find the ind's of vertices that are in use
    used = zeros(length(pcl),1); % pre-allocate
    used(iused) = 1; % create a map of used vertices
    map2used = cumsum(used); % conversion table from indices of old vertices to the new one
    pcl = pcl(iused,:); % remove the unused vertices
    J1l = J1l(iused,:);
    TRI = map2used(TRI); % update the ind's of vertices
    
    % create the 3D mesh
    TR = triangulation(TRI,double(pcl)); % create the object
    
    % visualize
    figure;
    TM = trimesh(TR); % plot the mesh
    set(TM,'FaceVertexCData',J1l); % set colors to input image
    set(TM,'Facecolor','interp');
    % set(TM,'FaceColor','red'); % if you want a colored surface
    set(TM,'EdgeColor','none'); % suppress the edges
    xlabel('x (mm)')
    ylabel('y (mm)')
    zlabel('z (mm)')
    axis([-250 250 -250 250 400 900])
    set(gca,'xdir','reverse')
    set(gca,'zdir','reverse')
    daspect([1,1,1])
    axis tight
    end
    