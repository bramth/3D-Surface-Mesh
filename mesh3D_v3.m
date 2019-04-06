clear all, clc, format compact, close all
warning('off','images:initSize:adjustingMag')
%% Open images 

im{1} = imread('..\..\IPCV_project3\subject4\subject4_Left\subject4_Left_1.jpg');
im{2} = imread('..\..\IPCV_project3\subject4\subject4_Middle\subject4_Middle_1.jpg');
im{3} = imread('..\..\IPCV_project3\subject4\subject4_Right\subject4_Right_1.jpg');

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

[im_ml{2},im_ml{1}] = rectifyStereoImages(im{2},im{1},stereoParams{1},'OutputView','full');
[im_mr{1},im_mr{2}] = rectifyStereoImages(im{2},im{3},stereoParams{2},'OutputView','full');

figure;
subplot(1,2,1);
imshow(stereoAnaglyph(im_ml{1},im_ml{2}));
subplot(1,2,2);
imshow(stereoAnaglyph(im_mr{1},im_mr{2}));

%% Disparity map

disp_start = 216;
disp_range = 40;
disparity_map{1} = create_disparity(im_ml{1},im_ml{2},disp_start,disp_range,false);
disparity_map{2} = create_disparity(im_mr{1},im_mr{2},disp_start,disp_range,false);


%% Obtain unreliables

for i = 1:lenth(disparity_map)
    unreliable{i} = (disparity_map{i}==-realmax('single'));
end

%% Reconstruct face
for i = 1:length(disparity_map)
    point_cloud{i} = create_point_cloud(disparity_map{i},stereoParams{i},false);
    %point_cloud{i} = pcdenoise(point_cloud{i});
end

%% Combine both point-clouds

[~,point_cloud{2}] = pcregistericp(point_cloud{2},point_cloud{1});
point_cloud_merge = pcmerge(point_cloud{1},point_cloud{2});

%% Obtain only location
 
% for i = 1:length(point_cloud)
%     pc_loc{i} = point_cloud{i}.Location;
% end

% DIT KAN NIET WANT HET GAAT NIET OVER ZELFDE OBJECT. KAN ALLEEN OVER
% IM(mid)
pc_merge_loc = point_cloud_merge.Location;

%%%% FROM HERE COPY PASTE %%%%

%% create a connectivity structure
[M, N] = size(disparity{i}); % get image size
res = 2; % resolution of mesh
[nI,mI] = meshgrid(1:res:N,1:res:M); % create a 2D meshgrid of pixels, thus defining a resolution grid
TRI = delaunay(nI(:),mI(:)); % create a triangle connectivity list
indI = sub2ind([M,N],mI(:),nI(:)); % cast grid points to linear indices

%% linearize the arrays and adapt to chosen resolution

pcl = reshape(pc_merge_loc,N*M,3); % reshape to (N*M)x3
im_ml_vect = reshape(im_ml{2},[N*M,3]); % reshape to (N*M)x3
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
view([0,90]);

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

function [disparity_map] = create_disparity(im1,im2,start,range,plotting)
    disparity_range = [start, start+16*range];
    disparity_map = disparity(rgb2gray(im1),rgb2gray(im2),'DisparityRange',disparity_range);
    if plotting == true
        figure
        imshow(disparityMap,disparityRange);
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