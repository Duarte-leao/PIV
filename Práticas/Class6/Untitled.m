clear 
clc
load PIV_plane.mat
%%
A = [xyzgood ones(size(xyzgood,1),1)];
[U,S,V] = svd(A, 'econ');

%sacar a ultima coluna de V
x = V(:,end)

%%
showPointCloud(pc2)