clear all
close all
%% Load data
im1=imread('rgb_image_10.png');
im2=imread('rgb_image_14.png');
im3=imread('rgb_image_17.png');
load depth_10.mat;
dep1=depth_array;
load depth_14.mat;
dep2=depth_array;
load depth_17.mat;
dep3=depth_array;
load calib_asus.mat;
K=Depth_cam.K;
Krgb=RGB_cam.K;

xyz=get_xyzasus(dep1(:),[480 640],1:480*640,K,1,0);