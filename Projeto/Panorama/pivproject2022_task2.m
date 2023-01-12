function pivproject2022_task2(varargin)

    p = inputParser;
    addRequired(p,'ref_image',@ischar);
    addRequired(p,'input_dir',@ischar);
    addRequired(p,'output_dir',@ischar);
    parse(p,varargin{:});
    ref_image = str2num(p.Results.ref_image);
    input_dir = p.Results.input_dir;
    output_dir = p.Results.output_dir;

    % Get a list of all files in the folder with the desired file name pattern.
    
    input_filePattern = fullfile(input_dir, '*.mat'); % Change to whatever pattern you need.
    Input_sift = dir(input_filePattern);
    images_filePattern = fullfile(input_dir, '*.jpg');
    images = dir(images_filePattern);
    
    imgs = imageDatastore(input_dir, "FileExtensions", ".jpg");
    montage(imgs.Files)
    numImages = numel(imgs.Files);
    tforms(numImages) = projective2d;
    
    image_fullFileName = fullfile(images(1).folder, images(1).name);
    image = imread(image_fullFileName);
    input_baseFileName = Input_sift(1).name;
    input_fullFileName = fullfile(Input_sift(1).folder, input_baseFileName);
    load(input_fullFileName);
    points = p;
%     features = des;
    features = d;
    if size(points,1) > size(points,2)
            points = points';
    end
   
    for k = 2 : numImages
        % Read I(n).
        I = readimage(imgs, k);

        % Convert image to grayscale.
        grayImage = im2gray(I);    

        % Save image size.
        imageSize(k,:) = size(grayImage);
        
        pointsPrevious = points;
        featuresPrevious = features;
        
        image_fullFileName = fullfile(images(k).folder, images(k).name);
        image = imread(image_fullFileName);
        input_baseFileName = Input_sift(k).name;
        input_fullFileName = fullfile(Input_sift(k).folder, input_baseFileName);
        load(input_fullFileName);
        points = p;
%         features = des;
        features = d;
        fprintf(1, 'Now reading %s\n', input_baseFileName);
        
        % Find correspondences between I(k) and I(k-1).
        indexPairs = NNeighbour(features, featuresPrevious);
%         ##### verificar no NN se estou a fazer match pelo vector que tem menos numeros ou ent removo duplicados
        if size(points,1) > size(points,2)
            points = points';
        end
        matchedPoints = points(:, indexPairs(:,1));
        matchedPointsPrev = pointsPrevious(:, indexPairs(:,2));        
        
        % Estimate the transformation between I(k) and I(k-1).
        K = ceil(log(1-0.99)/log(1-0.1^4));
%         K = 5000;
        [tforms(k),~,~] = RANSAC(matchedPoints, matchedPointsPrev, 0.5, K);
        
        
%         imOut = imwarp(image,projective2d(tforms(k).T'));
%         figure
%         imshow(imOut)
        
        
        % Compute T(1) * T(2) * ... * T(k-1) * T(k).
        tforms(k).T = tforms(k-1).T * tforms(k).T'; 
        
%         imOut = imwarp(image,projective2d(tforms(k).T'));
%         figure
%         imshow(imOut)
        
    end
%     Tinv = invert(tforms(ref_image));
%     for i = 1:numel(tforms)    
%         tforms(i).T = Tinv.T * tforms(i).T;
%     end
    for i = 1:numel(tforms)           
        [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(i,2)], [1 imageSize(i,1)]);    
    end
    avgXLim = mean(xlim, 2);
    [~,idx] = sort(avgXLim);
    centerIdx = floor((numel(tforms)+1)/2);
    centerImageIdx = idx(centerIdx);

    Tinv = invert(tforms(ref_image));
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
    
    
end

function [index_of_matching_p] = NNeighbour(di, dt)
    if size(di,1) < size(di,2)
        len = min(size(di,2), size(dt,2));
        index_of_matching_p = zeros(len, 2);
        index_of_matching_p(:,1) = [1:len]';
        for i = 1:len
            norm = vecnorm(di(:,i)-dt);
            index_of_matching_p(i,2) =  datasample(find(norm == min(norm)),1);
        end
    else
        len = min(size(di,1), size(dt,1));
        index_of_matching_p = zeros(len, 2);
        index_of_matching_p(:,1) = [1:len]';
        for i = 1:len
            norm = vecnorm(di(i,:)'-dt');
            index_of_matching_p(i,2) =  datasample(find(norm == min(norm)),1);
        end
    end
    
end

% 
% function [best_homography, im_points,temp_points] = RANSAC(im_points,temp_points, threshold, num_iter)
%     best_inliers = [0 0];
%     i=1;
%     while size(best_inliers,2) < size(im_points,2)*.065
%         rand_ind = datasample(1:size(im_points,2),4);
%         H = homography(im_points(:,rand_ind), temp_points(:,rand_ind));
%         inliers = find(distance(im_points, temp_points, H)<50);
%         i = i + 1;
%         if size(inliers,2) > size(best_inliers,2)
%             i
%             best_inliers = inliers;
%             if size(best_inliers,2) > size(im_points,2)*threshold
%                break 
%             end
%         end
%     end
%     im_points = im_points(:,best_inliers);
%     temp_points = temp_points(:,best_inliers);
%     best_homography = homography(im_points, temp_points);
% end
function [best_homography, im_points,temp_points] = RANSAC(im_points,temp_points, threshold, num_iter)
    best_inliers = [0 0];
    for i = 1:num_iter
        rand_ind = datasample(1:size(im_points,2),4);
        H = homography(im_points(:,rand_ind), temp_points(:,rand_ind));
        inliers = find(distance(im_points, temp_points, H)<4);

        if size(inliers,2) > size(best_inliers,2)
            i
            best_inliers = inliers;
            if size(best_inliers,2) > size(im_points,2)*threshold
               break 
            end
        end
    end
    im_points = im_points(:,best_inliers);
    temp_points = temp_points(:,best_inliers);
    best_homography = homography(im_points, temp_points);
end

function H = homography(im_points,temp_points)
    X = [im_points',ones(size(im_points',1),1)];
    M = zeros(2*size(im_points',1),9);
    for i = 1:size(im_points',1)
        M(2*i-1,1:3) = X(i,:);
        M(2*i,4:6) = X(i,:);
        M(2*i-1,7:9) = X(i,:);
        M(2*i,7:9) = X(i,:);
    end
    M(:,7:9) = M(:,7:9).*-repmat(temp_points(:), 1, 3);
    [~,~,V] = svd(M'*M);
    H = reshape(V(:,end),[3 3])'/V(end,end);
 
end

function dist = distance(p_im, p_temp, H)
    p_im = [p_im;ones(1,size(p_im,2))];
    p_temp = [p_temp;ones(1,size(p_im,2))];
    p_estimate = H*p_im;
    p_estimate = p_estimate./p_estimate(3,:);
    dist = vecnorm(p_temp-p_estimate);
end