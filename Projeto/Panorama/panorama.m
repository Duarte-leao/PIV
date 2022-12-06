close all
clear

%% Code

im_1 = imread('im1.jpeg');
im_2 = imread('im2.jpeg');
im_3 = imread('im3.jpeg');
load('im1.mat')
load('im2.mat')
load('im3.mat')
% img_1=rgb2gray(im_3);
% points = detectSURFFeatures(img_1, 'MetricThreshold', 20);
% [im_1_features, p_im1] = extractFeatures(img_1, points);
% p3 = p_im1.Location';
% d3 = im_1_features';
% save('im3.mat','d3','p3')

% figure 
% subplot(131);
% imagesc(im_1);
% hold on
% plot(p1(1,:), p1(2,:), 'r.')
% subplot(132);
% imagesc(im_2);
% hold on
% plot(p2(1,:), p2(2,:), 'g.')
% subplot(133);
% imagesc(im_3);
% hold on
% plot(p3(1,:), p3(2,:), 'b.')


p1 = round(p1);
p2 = round(p2);
p3 = round(p3);


calculate = 1; % 1 - homography; 2 - matches

switch calculate
    case 1
        load('indices1_2.mat')
%         load('indices3_2.mat')
        pi = p1;
        pt = p2;
        im_1 = im_1;
        tm = im_2;
        


        temp_points = [pt(1,indices(:,2)); pt(2,indices(:,2))];
        im_points = [pi(1,indices(:,1)); pi(2,indices(:,1))];

        tic;
        [best_homography, im_points,temp_points] = RANSAC(im_points,temp_points, 0.5, 10000);
        toc

        random_indices = datasample(1:size(im_points,2),4);
        figure
        subplot(121);
        imagesc(im_1);
        hold on
        plot(im_points(1,random_indices(1)), im_points(2,random_indices(1)), 'r.', 'MarkerSize',20)
        hold on
        plot(im_points(1,random_indices(2)), im_points(2,random_indices(2)), 'g.', 'MarkerSize',20)
        hold on
        plot(im_points(1,random_indices(3)), im_points(2,random_indices(3)), 'b.', 'MarkerSize',20)
        hold on
        plot(im_points(1,random_indices(4)), im_points(2,random_indices(4)), 'y.', 'MarkerSize',20)
        subplot(122);
        imagesc(tm);
        hold on
        plot(temp_points(1,random_indices(1)), temp_points(2,random_indices(1)), 'r.', 'MarkerSize',20)
        hold on
        plot(temp_points(1,random_indices(2)), temp_points(2,random_indices(2)), 'g.', 'MarkerSize',20)
        hold on
        plot(temp_points(1,random_indices(3)), temp_points(2,random_indices(3)), 'b.', 'MarkerSize',20)
        hold on
        plot(temp_points(1,random_indices(4)), temp_points(2,random_indices(4)), 'y.', 'MarkerSize',20)

        figure 
        subplot(121);
        imagesc(tm);
        hold on
        plot(temp_points(1,:), temp_points(2,:), 'r.')
        subplot(122);
        imagesc(im_1);
        hold on
        plot(im_points(1,:), im_points(2,:), 'g.')

        imOut = imwarp(im_1,projective2d(best_homography'));
        figure
        imshow(imOut)
        
        %% matlab panoramic image
        [xlim, ylim] = outputLimits(projective2d(best_homography'), [1 size(im_2,2)], [1 size(im_2,1)]);

        % Find the minimum and maximum output limits. 
        xMin = min([1; xlim(:)]);
        xMax = max([size(im_1,2); xlim(:)]);

        yMin = min([1; ylim(:)]);
        yMax = max([size(im_1,1); ylim(:)]);

        % Width and height of panorama.
        width  = round(xMax - xMin);
        height = round(yMax - yMin);

        % Initialize the "empty" panorama.
        pan = zeros([height width 3], 'like', im_1);

        blender = vision.AlphaBlender('Operation', 'Binary mask', ...
            'MaskSource', 'Input port');  

        % Create a 2-D spatial reference object defining the size of the panorama.
        xLimits = [xMin xMax];
        yLimits = [yMin yMax];
        panoramaView = imref2d([height width], xLimits, yLimits);

        % Create the panorama.
        warpedImage = imwarp(im_1, projective2d(best_homography'), 'OutputView', panoramaView);
        mask = imwarp(true(size(im_1,1),size(im_1,2)), projective2d(best_homography'), 'OutputView', panoramaView);
        pan = step(blender, pan, warpedImage, mask);
        warpedImage = imwarp(im_2, projective2d(eye(3)), 'OutputView', panoramaView);
        mask = imwarp(true(size(im_2,1),size(im_2,2)), projective2d(eye(3)), 'OutputView', panoramaView);
        pan = step(blender, pan, warpedImage, mask);

        figure
        imshow(pan)

    case 2
        
        di = d1;
%         di = d3;
        dt = d2;
        indices = zeros(size(di,2), 2);
        indices(:,1) = [1:size(di,2)]';
        tic;
        for i = 1:size(di,2)
            indices(i,2) = NNeighbour(di(:,i), dt);
        end
%         save('indices3_2.mat','indices')
%         save('indices1_2.mat','indices')
        toc
end

function [index_of_matching_p] = NNeighbour(di_point, dt)
    norm = vecnorm(di_point-dt);
%     min_norm = min(norm);
    index_of_matching_p = find(norm == min(norm));
    index_of_matching_p =  datasample(index_of_matching_p,1);
end