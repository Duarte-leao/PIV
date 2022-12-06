close all
clear

%% Code
load('rgb0001.mat')
pi = p;
di = d;
load('templateSNS.mat')
pt = p;
dt = d;
im = imread('rgb0001.jpg');
tm = imread('templateSNS.jpg');
% figure 
% subplot(121);
% imagesc(tm);
% hold on
% plot(pt(1,:), pt(2,:), 'r.')
% subplot(122);
% imagesc(im);
% hold on
% plot(pi(1,:), pi(2,:), 'g.')

load('indices.mat')
% indices_sorted = indices;

indices_sorted = sortrows(indices,[2 3]);
% [~,id] = unique(indices_sorted(:,2));

% indices_sorted = indices_sorted(id,:);

temp_points = [pt(1,indices_sorted(:,2)); pt(2,indices_sorted(:,2))];
im_points = [pi(1,indices_sorted(:,1)); pi(2,indices_sorted(:,1))];

tic;
[best_homography, im_points,temp_points] = RANSAC(im_points,temp_points, 0.5, 10000);
toc

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

% indices = zeros(size(di,2), 3);
% indices(:,1) = [1:size(di,2)]';
% tic;
% for i = 1:size(di,2)
%     [indices(i,2), indices(i,3)] = NNeighbour(di(:,i), dt);
% end
% save('indices.mat','indices')
% toc
% see if there are repeated values
% if length(indices) ~= length(unique(indices))
%     1
% end

function [index_of_matching_p, min_norm] = NNeighbour(di_point, dt)
    norm = vecnorm(di_point-dt);
    min_norm = min(norm);
    index_of_matching_p = find(norm == min(norm));
    index_of_matching_p =  datasample(index_of_matching_p,1);
end