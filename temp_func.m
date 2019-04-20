%% Function To find matched correspondances between frames and 
%% compute the Transformation Matrix

function [theta,translation,scale] = temp_func(frame_prev,frame_new)

    % Finding SURF points in both the Images (frame_prev and frame_new)
    points_Image1 = detectSURFFeatures(frame_prev); 
    points_Image2 = detectSURFFeatures(frame_new);

    % Extracting the features 
    [frame_1,valid_points_1] = extractFeatures(frame_prev,points_Image1);
    [frame_2,valid_points_2] = extractFeatures(frame_new,points_Image2);

    % Finding the locations of matched points.
    loc = matchFeatures(frame_1,frame_2) ;

    % Finding matched points.
    matched_Image1 = valid_points_1(loc(:, 1)); % frame_prev
    matched_Image2 = valid_points_2(loc(:, 2)); % frame_new

     %% Show Corresponding matches (might contain outliers)
%     figure;
%     showMatchedFeatures(frame_prev,frame_new,matched_Image1,matched_Image2);
%     legend('matched points 1','matched points 2');

    %% Define Geometric Transformation Objects using MSAC a variant of RANSAC
    [tform_matrix, matchedImage2_inliers, matchedImage1_inliers] = estimateGeometricTransform(matched_Image2, matched_Image1,'affine');

     %% Display Correspondances only inliers
%     figure;
%     showMatchedFeatures(frame_prev,frame_new,matchedImage1_inliers,matchedImage2_inliers);
%     title('Correspondances only inliers');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Extract scale and rotation part sub-matrix.
    
	% The output of estimateGeomet.. is transpose of actual transformation Matrix
	% Hence we need to have another transpose
    H = tform_matrix.T;
    [theta,translation,scale] = compute_param(H);

end
