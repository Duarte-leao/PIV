close all
clear all

%% Code
load('rgb0001.mat')
pi = p;
di = d;
load('templateSNS.mat')
pt = p;
dt = d;
im = imread('rgb0001.jpg');
tm = imread('templateSNS.jpg');
figure 
subplot(121);
imagesc(tm);
hold on
plot(pt(1,:), pt(2,:), 'r.')
subplot(122);
imagesc(im);
hold on
plot(pi(1,:), pi(2,:), 'g.')

load('indices.mat')

match_temp = [pt(1,indices); pt(2,indices)];

random_indices = datasample(1:size(pi,2),4);
figure
subplot(121);
imagesc(im);
hold on
plot(pi(1,random_indices(1)), pi(2,random_indices(1)), 'r.', 'MarkerSize',20)
hold on
plot(pi(1,random_indices(2)), pi(2,random_indices(2)), 'g.', 'MarkerSize',20)
hold on
plot(pi(1,random_indices(3)), pi(2,random_indices(3)), 'b.', 'MarkerSize',20)
hold on
plot(pi(1,random_indices(4)), pi(2,random_indices(4)), 'y.', 'MarkerSize',20)
subplot(122);
imagesc(tm);
hold on
plot(match_temp(1,random_indices(1)), match_temp(2,random_indices(1)), 'r.', 'MarkerSize',20)
hold on
plot(match_temp(1,random_indices(2)), match_temp(2,random_indices(2)), 'g.', 'MarkerSize',20)
hold on
plot(match_temp(1,random_indices(3)), match_temp(2,random_indices(3)), 'b.', 'MarkerSize',20)
hold on
plot(match_temp(1,random_indices(4)), match_temp(2,random_indices(4)), 'y.', 'MarkerSize',20)

figure 
subplot(121);
imagesc(tm);
hold on
plot(match_temp(1,:), match_temp(2,:), 'r.')
subplot(122);
imagesc(im);
hold on
plot(pi(1,:), pi(2,:), 'g.')

% indices = zeros(size(di,2), 1);

% tic;
% for i = 1:size(di,2)
%     indices(i,:) = NNeighbour(di(:,i), dt);
% end
% save('indices.mat','indices')
% toc
% % see if there are repeated values
% if length(indices) ~= length(unique(indices))
%     1
% end
function index_of_matching_p = NNeighbour(di_point, dt)
    norm = vecnorm(di_point-dt);
    index_of_matching_p = find(norm == min(norm));
    index_of_matching_p =  datasample(index_of_matching_p,1);
end