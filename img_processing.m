
%% Load reference image, and compute surf features
img_path="data/reference/can_ref_1.png";

ref_img=imread(img_path);
ref_img_gray = rgb2gray(ref_img);
ref_pts = detectSURFFeatures(ref_img_gray);
[ref_features,  ref_validPts] = extractFeatures(ref_img_gray,  ref_pts);
figure; imshow(ref_img);
hold on; plot(ref_pts.selectStrongest(50));
%% Visual 25 SURF features
figure;
subplot(5,5,3); title('First 25 Features');
for i=1:25
    scale = ref_pts(i).Scale;
    image = imcrop(ref_img,[ref_pts(i).Location-10*scale 20*scale 20*scale]);
    subplot(5,5,i);
    imshow(image);
    hold on;
    rectangle('Position',[5*scale 5*scale 10*scale 10*scale],'Curvature',1,'EdgeColor','g');
end

%% Compare to video frame
img_camera = imread('img_home.jpg');
I = rgb2gray(img_camera);

% Detect features
I_pts = detectSURFFeatures(I);
[I_features, I_validPts] = extractFeatures(I, I_pts);
figure;imshow(image);
hold on; plot(I_pts.selectStrongest(50));


% strle
% regionprops
% feature base computer vision toolbox
% identify and clasify
% template matching
% https://www.youtube.com/watch?v=9_6utqvsCtA




