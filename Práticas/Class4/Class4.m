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
indices = zeros(size(di,2), 1);

tic;
for i = 1:size(di,2)
    indices(i,:) = NNeighbour(di(:,i), dt);
end
toc
% see if there are repeated values
if length(indices) ~= length(unique(indices))
    1
end
function index_of_matching_p = NNeighbour(di_point, dt)
    norm = vecnorm(di_point-dt);
    index_of_matching_p = find(norm == min(norm));
    index_of_matching_p = index_of_matching_p(1);
end