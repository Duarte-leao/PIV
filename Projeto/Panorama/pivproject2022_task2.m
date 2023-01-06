function pivproject2022_task2(ref_image, path_to_input_folder, path_to_output_folder)
    % Specify the folder where the files live.
%     myFolder = 'D:\IST\MEEC\4º Ano\P2\PIV\Projeto\Homography\images';
    
    ref_image = ref_image;
    input_folder = path_to_input_folder;
    output_folder =  path_to_output_folder;
    % Get a list of all files in the folder with the desired file name pattern.
    
    input_filePattern = fullfile(input_folder, '*.mat'); % Change to whatever pattern you need.
    Input_sift = dir(input_filePattern);
    images_filePattern = fullfile(input_folder, '*.jpg');
    images = dir(images_filePattern);
    
    imgs = imageDatastore(input_folder, "FileExtensions", ".jpg");

    numImages = numel(imgs.Files);
    tforms(numImages) = projective2d;
    
    image_fullFileName = fullfile(images(1).folder, images(1).name);
    image = imread(image_fullFileName);
    input_baseFileName = Input_sift(1).name;
    input_fullFileName = fullfile(Input_sift(1).folder, input_baseFileName);
    load(input_fullFileName);
    points = p;
    features = des;
   
    for k = 2 : numImages
        pointsPrevious = points;
        featuresPrevious = features;
        
        image_fullFileName = fullfile(images(k).folder, images(k).name);
        image = imread(image_fullFileName);
        input_baseFileName = Input_sift(k).name;
        input_fullFileName = fullfile(Input_sift(k).folder, input_baseFileName);
        load(input_fullFileName);
        points = p;
        features = des;
        fprintf(1, 'Now reading %s\n', input_baseFileName);
        
        % Find correspondences between I(k) and I(k-1).
        indexPairs = NNeighbour(features, featuresPrevious);
        ##### verificar no NN se estou a fazer match pelo vector que tem menos numeros ou ent removo duplicados
        
        matchedPoints = points(indexPairs(:,1), :);
        matchedPointsPrev = pointsPrevious(indexPairs(:,2), :);        
        
        % Estimate the transformation between I(k) and I(k-1).
        K = ceil(log(1-0.99)/log(1-0.2^4));
        [tforms(k),~,~] = RANSAC(matchedPoints, matchedPointsPrev, 0.5, K);

        % Compute T(1) * T(2) * ... * T(k-1) * T(k).
        tforms(k).T = tforms(k-1).T * tforms(k).T'; 
        
    end
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


function [best_homography, im_points,temp_points] = RANSAC(im_points,temp_points, threshold, num_iter)
    best_inliers = [0 0];
    i=1;
    while size(best_inliers,2) < size(im_points,2)*.015
        rand_ind = datasample(1:size(im_points,2),4);
        H = homography(im_points(:,rand_ind), temp_points(:,rand_ind));
        inliers = find(distance(im_points, temp_points, H)<5);
        i = i + 1;
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