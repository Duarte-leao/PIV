function pivproject2022_task1(varargin)
    p = inputParser;
    addRequired(p,'template_dir',@ischar);
    addRequired(p,'input_dir',@ischar);
    addRequired(p,'output_dir',@ischar);
    parse(p,varargin{:});
    template_dir = p.Results.template_dir;
    input_dir = p.Results.input_dir;
    output_dir = p.Results.output_dir;

    % Specify the folder where the files live.
%     myFolder = 'D:\IST\MEEC\4º Ano\P2\PIV\Projeto\Homography\images';
%     args = argv();
%     template_dir = "D:/IST/MEEC/4º Ano/P2/PIV/Projeto/Homography/template"; 
%     input_dir = "D:/IST/MEEC/4º Ano/P2/PIV/Projeto/Homography/input";
%     output_dir = "D:/IST/MEEC/4º Ano/P2/PIV/Projeto/Homography/output";

%     args = argv();
%     template_dir = args{1}; 
%     input_dir = args{2};
%     output_dir = args{3};

%     % Check to make sure that folder actually exists.  Warn user if it doesn't.
%     if ~isfolder(input_folder)
%         errorMessage = sprintf('Error: The following folder does not exist:\n%s\nPlease specify a new folder.', input_folder);
%         uiwait(warndlg(errorMessage));
%         input_folder = uigetdir(); % Ask for a new one.
%         if input_folder == 0
%              % User clicked Cancel
%              return;
%         end
%     end
    % Get a list of all files in the folder with the desired file name pattern.
    template_filePattern = fullfile(template_dir, '*.mat'); % Change to whatever pattern you need.
    Template_sift = dir(template_filePattern);
    input_filePattern = fullfile(input_dir, '*.mat'); % Change to whatever pattern you need.
    Input_sift = dir(input_filePattern);
    images_filePattern = fullfile(input_dir, '*.jpg');
    images = dir(images_filePattern);
    template_baseFileName = Template_sift.name;
    template_fullFileName = fullfile(Template_sift.folder, template_baseFileName);
    load(template_fullFileName);
    pt = p;
    dt = d;    
    for k = 1 : length(Input_sift)
        image_fullFileName = fullfile(images(k).folder, images(k).name);
        image = imread(image_fullFileName);
        input_baseFileName = Input_sift(k).name;
        input_fullFileName = fullfile(Input_sift(k).folder, input_baseFileName);
        load(input_fullFileName);
        pi = p;
        di = d;
        fprintf(1, 'Now reading %s\n', input_baseFileName);
        indexPairs = NNeighbour(di,dt);
%         save('index_pairs.mat','indexPairs')
        temp_points = [pt(1,indexPairs(:,2)); pt(2,indexPairs(:,2))];
        im_points = [pi(1,indexPairs(:,1)); pi(2,indexPairs(:,1))];
%         H = RANSAC(im_points,temp_points, 0.5, 10000);
        H = RANSAC(im_points,temp_points, 0.5, 1000);
        save(strcat(output_dir, strcat('\H_', input_baseFileName(end-7:end))),'H')
        
        imOut = imwarp(image,projective2d(H'));
        figure;
        imshow(imOut)
        
    end
end

function [index_of_matching_p] = NNeighbour(di, dt)
    len = min(size(di,2), size(dt,2));
    index_of_matching_p = zeros(len, 2);
    index_of_matching_p(:,1) = [1:len]';
    for i = 1:len
        norm = vecnorm(di(:,i)-dt);
        index_of_matching_p(i,2) =  datasample(find(norm == min(norm)),1);
    end
end

% function [best_homography, im_points,temp_points] = RANSAC(im_points,temp_points, threshold, num_iter)
%     best_inliers = [0 0];
%     for i = 1:num_iter
%         rand_ind = datasample(1:size(im_points,2),4);
%         H = homography(im_points(:,rand_ind), temp_points(:,rand_ind));
%         inliers = find(distance(im_points, temp_points, H)<5);
% 
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

function [best_homography] = RANSAC(im_points,temp_points, threshold, num_iter)
    best_inliers = [0 0];
    i=1;
    while size(best_inliers,2) < size(im_points,2)*0.025
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