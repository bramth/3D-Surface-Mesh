clear all,  clc, format compact, close all
warning('off','images:initSize:adjustingMag')

%% Open images 

im{1} = imread('subject4\subject4_Left\subject4_Left_1.jpg');
im{2} = imread('subject4\subject4_Middle\subject4_Middle_1.jpg');
im{3} = imread('subject4\subject4_Right\subject4_Right_1.jpg');

for i = 1:length(im)
    im{i} = im2double(im{i});
end

im{3} = im{3}(:,150:end,:);

%% Calibration camera

% cameraParams{1}=Cam_calib2_left;
% cameraParams{2}=Cam_calib2_middle;
% cameraParams{3}=Cam_calib2_right;

% save camParam cameraParams

% load camParam

%% determine stereoparameters

% stereoParams{1}=Cam_calib2_lm;
% stereoParams{2}=Cam_calib2_mr;

% save stereoParam stereoParams

load stereoParam

%% Apply camera calibration to images

% for ii = 1:3
%     [im{ii},~] = undistortImage(im{ii},cameraParams{ii},'OutputView', 'same');
% end

%% Histogram matching

im{1} = imhistmatch(im{1},im{2});
im{3} = imhistmatch(im{3},im{2});

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

figure;
subplot(1,2,1);
imshow(stereoAnaglyph(im_lm{1},im_lm{2}));
subplot(1,2,2);
imshow(stereoAnaglyph(im_mr{1},im_mr{2}));


%% Filter image

h = fspecial('gaussian',5,1);
im_lm{1} = imfilter(im_lm{1},h);
im_lm{2} = imfilter(im_lm{2},h);
im_mr{1} = imfilter(im_mr{1},h);
im_mr{2} = imfilter(im_mr{2},h);


%% Disparity map

disp_range = 16*[-35,-10]; %30; %40;
disparity_map{1} = create_disparity(im_lm{2},im_lm{1},disp_range,true);

disp_range = 16*[10,35];
disparity_map{2} = create_disparity(im_mr{1},im_mr{2},disp_range,true);

%% Obtain unreliables

% for i = 1:length(disparity_map)
%     %unreliable{i} = (disparity_map{i}==-realmax('single')); %+ (pc_loc{i} == -inf) + (pc_loc{i} == inf))>0; %+ (pc_loc{i}(:,:,1)>250) + (pc_loc{i}(:,:,1)<-200) + (pc_loc{i}(:,:,2)>=175) + (pc_loc{i}(:,:,2)<=-180) + (pc_loc{i}(:,:,3)>=-410))>0;
% end

unreliable{1} = (disparity_map{1}==-realmax('single')) | (1-rgb2gray(im_lm{1})>0);
unreliable{2} = (disparity_map{2}==-realmax('single')) | (1-rgb2gray(im_lm{2})>0);

%unreliable{1} = unreliable{1} + (pc_loc{1} == -inf) + (pc_loc{1} == inf); %+ (pc_loc{1}(:,:,1)>173) + (pc_loc{1}(:,:,1)<-95) + (pc_loc{1}(:,:,2)>=165) + (pc_loc{1}(:,:,2)<=-170) + (pc_loc{1}(:,:,3)>=-400);
%unreliable{2} = unreliable{2} + (pc_loc{2} == -inf) + (pc_loc{2} == inf); %+ (pc_loc{2}(:,:,1)>250) + (pc_loc{2}(:,:,1)<-200) + (pc_loc{2}(:,:,2)>=175) + (pc_loc{2}(:,:,2)<=-180) + (pc_loc{2}(:,:,3)>=-410);


%% Reconstruct face
for i = 1:length(disparity_map)
    point_cloud{i} = create_point_cloud(disparity_map{i},stereoParams{i},true);
    %point_cloud{i} = pcdenoise(point_cloud{i});
end

%% Combine both point-clouds

%[~,point_cloud{2},pc_rms_error] = pcregistericp(point_cloud{2},point_cloud{1},'Verbose',true);
%point_cloud_merge = pcmerge(point_cloud{1},point_cloud{2});

%% Obtain only location
 
for i = 1:length(point_cloud)
    pc_loc{i} = point_cloud{i}.Location;
end

%pc_merge_loc = point_cloud_merge.Location;


%% Polish disparity map



%%%% FROM HERE COPY PASTE %%%%


%% create a connectivity structure
[M, N] = size(disparity_map{2}); % get image size
res = 2; % resolution of mesh
[nI,mI] = meshgrid(1:res:N,1:res:M); % create a 2D meshgrid of pixels, thus defining a resolution grid
TRI = delaunay(nI(:),mI(:)); % create a triangle connectivity list
indI = sub2ind([M,N],mI(:),nI(:)); % cast grid points to linear indices

%% linearize the arrays and adapt to chosen resolution

%pcl = reshape(pc_merge_loc,N*M,3); % reshape to (N*M)x3
pcl = reshape(pc_loc{2},N*M,3); % reshape to (N*M)x3
im_ml_vect = reshape(im_mr{1},[N*M,3]); % reshape to (N*M)x3
pcl = pcl(indI,:); % select 3D points that are on resolution grid
im_ml_vect = im_ml_vect(indI,:); % select pixels that are on the resolution grid

%% remove the unreliable points and the associated triangles

ind_unreliable = find(unreliable{2}(indI));% get the linear indices of unreliable 3D points
imem = ismember(TRI(:),ind_unreliable); % find indices of references to unreliable points
[ir,~] = ind2sub(size(TRI),find(imem)); % get the indices of rows with refs to unreliable points.
TRI(ir,:) = []; % dispose them
iused = unique(TRI(:)); % find the ind's of vertices that are in use
used = zeros(length(pcl),1); % pre-allocate
used(iused) = 1; % create a map of used vertices
map2used = cumsum(used); % conversion table from indices of old vertices to the new one
pcl = pcl(iused,:); % remove the unused vertices
im_ml_vect = im_ml_vect(iused,:);
TRI = map2used(TRI); % update the ind's of vertices

%% create the 3D mesh

TR = triangulation(TRI,double(pcl)); % create the object

%% visualize
figure(5), clf(5), hold on
TM = trimesh(TR); % plot the mesh
set(TM,'FaceVertexCData',im_ml_vect); % set colors to input image
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
%view([0,90]);

%% Match

% triangulateMultiview

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

    bs = 15;        %defauld bs=15
    cTH = 0.7;      %default 0.5
    uTH = 15;       %default 15
    tTH = 0.0000;   %default 0.0002 only applies if method is blockmatching
    dTH = 15;       %default []

%    disparity_map = disparity(rgb2gray(im1),rgb2gray(im2),'DisparityRange',disparity_range);
    disparity_map = disparity(rgb2gray(im1),rgb2gray(im2),'DisparityRange',disparity_range,...
        'ContrastThreshold',cTH, 'UniquenessThreshold',uTH, 'DistanceThreshold',dTH,'BlockSize',bs);
    if plotting == true
        figure
        imshow(disparity_map,disparity_range);
        colorbar
    end
end

function [point_cloud] = create_point_cloud(disparity_map,stereoParams,plotting)
    xyzPoints = reconstructScene(disparity_map,stereoParams);
    point_cloud = pointCloud(xyzPoints);
    if plotting == true
        figure;
        pcshow(xyzPoints);
    end
end
    
% function [D]=stereomatchingSSD(im1,im2,Dmax)
%     d=[1:Dmax];
%     for m=1:size(im1,2)
%         for n=1:size(im1,1)
%             D(n,m)=min(im1(n,m)-im2(max(1,n-d),m));
%         end
%     end
% end