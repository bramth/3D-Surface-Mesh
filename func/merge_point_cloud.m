function [point_cloud_merge,pc_rms_error] = merge_point_cloud(point_cloud,point_cloud_down,stereoParams,plotting)

    tform_pred = affine3d();
    tform_pred.T(1:3, 1:3) =  stereoParams{2}.RotationOfCamera2 * stereoParams{1}.RotationOfCamera2;
    [tform,~,pc_rms_error] = pcregistericp(point_cloud_down{2},point_cloud_down{1},'Verbose',false,'InitialTransform', tform_pred);
    point_cloud_merge = pcmerge(point_cloud{1},pctransform(point_cloud{2},tform),10);
    
    point_cloud_merge = pointCloud(point_cloud_merge.Location(find(point_cloud_merge.Location(:, 3) < 550), :));
    point_cloud_merge = pointCloud(point_cloud_merge.Location(find(point_cloud_merge.Location(:, 3) > 400), :));
    if plotting == true
        figure;
        pcshow(point_cloud_merge);
        view([0,-90])
        title('Merged PC')
    end
end