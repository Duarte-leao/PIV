%%%%% ver se consigo transpor a homography apenas no final


clear 
close all
%% Code
tic;
% Load images.
% buildingDir = fullfile(toolboxdir('vision'),'visiondata','building');
% imgs = imageDatastore(buildingDir);

% imgs = imageDatastore(["D:\IST\MEEC\4º Ano\P2\PIV\Projeto\Panorama\input\im1.jpeg"; "D:\IST\MEEC\4º Ano\P2\PIV\Projeto\Panorama\input\im2.jpeg"; "D:\IST\MEEC\4º Ano\P2\PIV\Projeto\Panorama\input\im3.jpeg"]);
% imgs = imageDatastore(["D:\IST\MEEC\4º Ano\P2\PIV\Projeto\Panorama\input\im2.jpeg"; "D:\IST\MEEC\4º Ano\P2\PIV\Projeto\Panorama\input\im3.jpeg"]);
imgs = imageDatastore("D:\IST\MEEC\4º Ano\P2\PIV\Projeto\Panorama\input\","FileExtensions",".jpeg");
% Display images to be stitched.
montage(imgs.Files)

% Read the first image from the image set.

I = readimage(imgs,1);

% Initialize features for I(1)
grayImage = im2gray(I);
points = detectSURFFeatures(grayImage, 'MetricThreshold', 300);
[features, points] = extractFeatures(grayImage,points);

% Initialize all the transformations to the identity matrix. Note that the
% projective transformation is used here because the building images are fairly
% close to the camera. For scenes captured from a further distance, you can use
% affine transformations.
numImages = numel(imgs.Files);
tforms(numImages) = projective2d;

% Initialize variable to hold image sizes.
imageSize = zeros(numImages,2);

% Iterate over remaining image pairs
for n = 2:numImages
    % Store points and features for I(n-1).
    pointsPrevious = points;
    featuresPrevious = features;
        
    % Read I(n).
    I = readimage(imgs, n);
    
    % Convert image to grayscale.
    grayImage = im2gray(I);    
    
    % Save image size.
    imageSize(n,:) = size(grayImage);
    
    % Detect and extract SURF features for I(n).
    points = detectSURFFeatures(grayImage, 'MetricThreshold', 300);    
    [features, points] = extractFeatures(grayImage, points);
  
    % Find correspondences between I(n) and I(n-1).
    indexPairs = NNeighbour(features', featuresPrevious');

    
    matchedPoints = points.Location(indexPairs(:,1), :)';
    matchedPointsPrev = pointsPrevious.Location(indexPairs(:,2), :)';        
    
    % Estimate the transformation between I(n) and I(n-1).
    K = ceil(log(1-0.99)/log(1-0.2^4));
    [tforms(n),~,~] = RANSAC(matchedPoints, matchedPointsPrev, 0.5, K);
    
    % Compute T(1) * T(2) * ... * T(n-1) * T(n).
    tforms(n).T = tforms(n-1).T * tforms(n).T'; 
end

% Compute the output limits for each transformation.
for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(i,2)], [1 imageSize(i,1)]);    
end

avgXLim = mean(xlim, 2);
[~,idx] = sort(avgXLim);
centerIdx = floor((numel(tforms)+1)/2);
centerImageIdx = idx(centerIdx);

Tinv = invert(tforms(centerImageIdx));
for i = 1:numel(tforms)    
    tforms(i).T = Tinv.T * tforms(i).T;
end

for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(i,2)], [1 imageSize(i,1)]);
end

maxImageSize = max(imageSize);

% Find the minimum and maximum output limits. 
xMin = min([1; xlim(:)]);
xMax = max([maxImageSize(2); xlim(:)]);

yMin = min([1; ylim(:)]);
yMax = max([maxImageSize(1); ylim(:)]);

% Width and height of panorama.
width  = round(xMax - xMin);
height = round(yMax - yMin);

% Initialize the "empty" panorama.
panorama = zeros([height width 3], 'like', I);

blender = vision.AlphaBlender('Operation', 'Binary mask', ...
    'MaskSource', 'Input port');  

% Create a 2-D spatial reference object defining the size of the panorama.
xLimits = [xMin xMax];
yLimits = [yMin yMax];
panoramaView = imref2d([height width], xLimits, yLimits);

% Create the panorama.
for i = 1:numImages
    
    I = readimage(imgs, i);   
   
    % Transform I into the panorama.
    warpedImage = imwarp(I, tforms(i), 'OutputView', panoramaView);
                  
    % Generate a binary mask.    
    mask = imwarp(true(size(I,1),size(I,2)), tforms(i), 'OutputView', panoramaView);
    
    % Overlay the warpedImage onto the panorama.
    panorama = step(blender, panorama, warpedImage, mask);
end

figure
imshow(panorama)
toc

function [index_of_matching_p] = NNeighbour(di, dt)
    len = min(size(di,2), size(dt,2));
    index_of_matching_p = zeros(len, 2);
    index_of_matching_p(:,1) = [1:len]';
    for i = 1:len
        norm = vecnorm(di(:,i)-dt);
        index_of_matching_p(i,2) =  datasample(find(norm == min(norm)),1);
    end
end