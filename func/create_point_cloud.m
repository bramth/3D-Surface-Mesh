function [point_cloud,xyzPoints,point_cloud_down] = create_point_cloud(disparity_map,stereoParams,plotting)
    xyzPoints = reconstructScene(disparity_map,stereoParams);
    point_cloud = pointCloud(xyzPoints);
    %point_cloud = removeInvalidPoints(point_cloud);
    %point_cloud = pcdenoise(point_cloud,'Threshold',0.1);
    point_cloud_down = pcdownsample(point_cloud,'gridAverage',10);

    if plotting == true
        figure;
        pcshow(xyzPoints);
    end
end