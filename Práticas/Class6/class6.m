% close all
clear

load("PIV_plane.mat")

figure(1)
imshow(im)

figure(2)
pcshow(pc)

figure(4)
pcshow(pc2) % pc2 são os pontos do chão

%% LEAST SQUARES plano
% pc2 são os pontos do chão
% least squares:
A = [xyzgood ones(length(xyzgood), 1)];

[U,S,V] = svd(A, 'econ');

a = V(1,end);
b = V(2,end);
c = V(3,end);
d = V(4,end);

%o chão é o plano "ax+by+cz+d=0"

%% RANSAC
clear
close all

load("PIV_plane_complete.mat")
% N data points
% repeat k times:
%     1 randomly select n samples (n<<N)
%     2 fit the model to the n points
%     3 determining the inliers ( e = ||f(x)-y||^2 < m )
% 4 Select the best model (most # inliers)
% 5 Re-fit the model to all inliers

N = length(xyz);
n = 4; %minimum number of points for a plane (4 unknows)
m = 1e-3;
p = 0.2; %worst case scenario (about 50% of the points in the image are on the floor
k = log(1-p)/log(1-p^n); %supostamente isto garante alguma coisa
k = ceil(k); %para ser inteiro

% mexer nos parametros "p" e "m" e talvez "n" e ver o que muda
% prof tinha dito p=0.5 e n=4; mas o k fica demasiado pequeno (11)
% o valor de "m" não me lembro, está um bocado ao calhas

number_inliers = zeros(k,1);
is_inlier = logical( zeros(k,N));

for i=1:k
    % passo 1 selecionar n pontos
    point_indices =  randperm(length(xyz), n);
    points = xyz(point_indices, :);

    % passo 2, fit model
    A = [points ones(n, 1)];

    [~,~,V] = svd(A, 'econ');
    
    a = V(1,end);
    b = V(2,end);
    c = V(3,end);
    d = V(4,end);

    f = @(x,y,z) a*x+b*y+c*z+d; 

    % passo 3 contar os inliers
    e = abs( f(xyz(:,1), xyz(:,2), xyz(:,3)) ).^2;
    is_inlier(i,:) = e<m;
    number_inliers(i,1) = length(e(e<m));
end

%passo 4 - modelo com mais inliers
[max, i] = max(number_inliers);

%passo 5 - modelo final feito apenas com os inliers
points = xyz( is_inlier(i,:), :);

A = [points ones(length(points), 1)];

[~,~,V] = svd(A, 'econ');

a = V(1,end);
b = V(2,end);
c = V(3,end);
d = V(4,end);


%inliers a vermelho
pc_in = pointCloud(  xyz( is_inlier(i,:), :) );
pc_in.Color = uint8([255 0 0] .* ones(max,1));

%outliers a cinzento
pc_out= pointCloud( xyz( ~is_inlier(i,:), :) );
pc_out.Color= uint8([100 100 100] .* ones(N-max,1));

pcshow(pc_in)
hold on
pcshow(pc_out)



