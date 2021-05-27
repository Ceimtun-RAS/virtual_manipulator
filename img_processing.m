
%% Load reference image, and compute surf features
clc, clear, close all
ref_img_path="data/reference/can_ref_1.png";
ref_img_path="data/reference/bottle_ref_1.png";

cam_img_path="img_home.jpg";
cam_img_path="image4.png";



ref_img=imread(ref_img_path);
ref_img_gray = rgb2gray(ref_img);
ref_pts = detectSURFFeatures(ref_img_gray);
[ref_features,  ref_validPts] = extractFeatures(ref_img_gray,  ref_pts);
figure; imshow(ref_img);
hold on; plot(ref_pts.selectStrongest(50));
%% Compare to video frame
cam_img = imread(cam_img_path);
I = rgb2gray(cam_img);

% Detect features
I_pts = detectSURFFeatures(I);
[I_features, I_validPts] = extractFeatures(I, I_pts);
figure;imshow(cam_img);
hold on; plot(I_pts.selectStrongest(50));






index_pairs = matchFeatures(ref_features, I_features);
ref_matched_pts = ref_validPts(index_pairs(:,1)).Location;
I_matched_pts = I_validPts(index_pairs(:,2)).Location;

figure, showMatchedFeatures(cam_img, ref_img, I_matched_pts, ref_matched_pts, 'montage');
title('Showing all matches');

%% Define Geometric Transformation Objects
gte = vision.GeometricTransformEstimator; 
gte.Method = 'Random Sample Consensus (RANSAC)';
[tform_matrix, inlierIdx] = step(gte, ref_matched_pts, I_matched_pts);



% strle
% regionprops
% feature base computer vision toolbox
% identify and clasify
% template matching
% https://www.youtube.com/watch?v=9_6utqvsCtA
