% close all
% clear
% clc

%% Code
%load('input/rgb0001.mat')
load('input/rgb0014.mat')
%load('input/Quarto/1.mat')
pi = p;
di = d;
load('template/templateSNS.mat')
% load('input/Quarto/2.mat')
pt = p;
dt = d;
%im = imread('input/rgb0001.jpg');
im = imread('input/rgb0014.jpg');
% im = imread('input/Quarto/1.jpg');
tm = imread('template/templateSNS.jpg');
% tm = imread('input/Quarto/2.jpg');

figure 
subplot(121);
imagesc(tm);
hold on
plot(pt(1,:), pt(2,:), 'r.')
subplot(122);
imagesc(im);
hold on
plot(pi(1,:), pi(2,:), 'g.')

calculate = 1; % 1 - homography; 2 - matches

switch calculate
    case 1
        %load('indices0001.mat')
        load('indices0014.mat')
        % indices_sorted = indices;

        % [~,id] = unique(indices_sorted(:,2));

        % indices_sorted = indices_sorted(id,:);

        temp_points = [pt(1,indices(:,2)); pt(2,indices(:,2))];
        im_points = [pi(1,indices(:,1)); pi(2,indices(:,1))];
        
              
        tic;
        [best_homography, im_points,temp_points] = RANSAC(im_points,temp_points, 0.5, 10000);
        toc
        
        error1 = cost_function(best_homography, im_points, temp_points);

        final_homography = Homography_LS(best_homography, im_points,temp_points);
        
        error2 = cost_function(final_homography, im_points, temp_points);
        
        random_indices = datasample(1:size(im_points,2),4);
        figure
        subplot(121);
        imagesc(im);
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
        imagesc(im);
        hold on
        plot(im_points(1,:), im_points(2,:), 'g.')
        
        imOut = imwarp(im,projective2d(best_homography'));
        imOut2 = imwarp(im,projective2d(final_homography'));
        figure(68)
        imshow(imOut)
        figure(69)
        imshow(imOut2)
        
        mean(error1)
        mean(error2)
        

        
    case 2
        indices = zeros(size(di,2), 3);
        indices(:,1) = [1:size(di,2)]';
        tic;
        for i = 1:size(di,2)
            [indices(i,2), indices(i,3)] = NNeighbour(di(:,i), dt);
        end
        %save('indices0001.mat','indices')
        save('indices0014.mat','indices')
        toc
end

function [index_of_matching_p, min_norm] = NNeighbour(di_point, dt)
    norm = vecnorm(di_point-dt);
    min_norm = min(norm);
    index_of_matching_p = find(norm == min(norm));
    index_of_matching_p =  datasample(index_of_matching_p,1);
end