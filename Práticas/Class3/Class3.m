% find projective matrix by minimizing algebraic error(svd eigenvector associated with lowest eigenvalue)
% run QR decomposition to find R and K and then T
clear all
close all
%% Code
im1=imread('rgb_image_10.png');
im2=imread('rgb_image_17.png');
load depth_10.mat;
dep1=depth_array;
load depth_17.mat;
dep2=depth_array;
load calib_asus.mat;
load pts_10_17.mat
K=Depth_cam.K;
Krgb=RGB_cam.K;

% %image 10
% xyz=get_xyzasus(dep1(:),[480 640],1:480*640,K,1,0);
% imd=get_rgbd(xyz,im1,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
% pc=pointCloud(xyz,"Color",reshape(imd,480*640,3));
% pc=pcdownsample(pc,"random",.4);
% 
% showPointCloud(pc);
% view([3.88 -90.00]);
% 
% %image 14 - or 17 if you click new points
% xyz2=get_xyzasus(dep2(:),[480 640],1:480*640,K,1,0);
% imd2=get_rgbd(xyz2,im2,R_d_to_rgb,T_d_to_rgb,RGB_cam.K);
% pc2=pointCloud(xyz2,"Color",reshape(imd2,480*640,3));
% pc2=pcdownsample(pc2,"random",.4);
% 
% showPointCloud(pc2);
% view([3.88 -90.00]);
% 
% imshow([im1 im2])
% imshow([imd imd2]);
% 
% %get corresponding points in two images. 
% % Initial set already defined and loaded (pts_10_17.mat) 
% % click in a fiew points and get their 3D
% %[u v]=ginput(n);
% %inds=sub2ind(size(dep1),v,u);
% %p=[u v];
% %xyz1=xyz(inds1,:);
% imagesc(imd);
% hold on;plot(p1(:,1),p1(:,2),'*r');
% 
% hold off;
% showPointCloud(pc);
% hold on
% plot3(xyz1(:,1),xyz1(:,2),xyz1(:,3),'or',"MarkerSize",12);
% axis equal;hold off;
% view([4.14 -90.00])

P = Projective_mat(p2, xyz1);


intrinsics=cameraIntrinsics([Krgb(1,1) Krgb(2,2)],[Krgb(1,3) Krgb(2,3)],[480 640]);
[R,T]=estimateWorldCameraPose(p2,xyz1,intrinsics,'MaxReprojectionError',20);
% [Q,R_1] = qr(P(1:3,1:3));

% R_2 = Q';
% T_2 = R_1*P(1:3,4)
R_1 = Krgb\P(1:3,1:3)
T_1 = Krgb\P(1:3,4)
% R_1 = (P(1:3,1:3)\Krgb)'
% % R_1 = inv(P(1:3,1:3)\Krgb)
% T_1 = Krgb\P(1:3,4)

% showPointCloud(pc);hold on;
% plotCamera('Size',.23,'Orientation',R,'Location',T);
% plotCamera('Size',.23,'Orientation',eye(3),'Location',zeros(3,1));
% 
% view([6.20 -39.32])

function P = Projective_mat(p,xyz)
    X = [xyz,ones(size(xyz,1),1)];
    p = p';
    M = zeros(2*size(xyz,1),12);
    for i = 1:6
        M(2*i-1,1:4) = X(i,:);
        M(2*i,5:8) = X(i,:);
        M(2*i-1,9:12) = X(i,:);
        M(2*i,9:12) = X(i,:);
    end
    M(:,9:12) = M(:,9:12).*repmat(p(:), 1, 4);
    [V,~] = svd(M'*M);
    P = reshape(V(:,12),[3 4])/V(12,12);
end