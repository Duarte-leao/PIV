import matplotlib.pyplot as plt
from skimage.io import imread
import numpy as np
import scipy.io
# from Ransac import *

def RANSAC(im_points, temp_points, threshold, num_iter):
    best_homography = np.zeros((3,3))
    max_inliers = 0
    
    for i in range(num_iter):
        # Select 4 random points
        random_indices = np.random.choice(im_points.shape[1], size=4, replace=False)
        A = []
        b = []
        for j in range(4):
            x, y = im_points[:,random_indices[j]]
            u, v = temp_points[:,random_indices[j]]
            A.append([x, y, 1, 0, 0, 0, -u*x, -u*y, -u])
            A.append([0, 0, 0, x, y, 1, -v*x, -v*y, -v])
            b.append(u)
            b.append(v)
        A = np.array(A)
        b = np.array(b)
        x = np.linalg.lstsq(A, b, rcond=None)[0]
        H = np.vstack((x, np.array([0, 0, 1])))
        projected_points = H.dot(np.vstack((im_points, np.ones(im_points.shape[1]))))
        projected_points = projected_points / projected_points[2]
        inliers = np.sum(np.abs(projected_points[:2] - temp_points) < threshold)
        if inliers > max_inliers:
            max_inliers = inliers
            best_homography = H
            im_points = im_points[:,np.abs(projected_points[:2] - temp_points) < threshold]
            temp_points = temp_points[:,np.abs(projected_points[:2] - temp_points) < threshold]
            
    return best_homography, im_points, temp_points

# load('rgb0001.mat')
mat1 = scipy.io.loadmat("rgb0001.mat")
p1 = mat1["p"]
d1 = mat1["d"]

# load('rgb0014.mat')
mat2 = scipy.io.loadmat("rgb0014.mat")
p2 = mat2["p"]
d2 = mat2["d"]

# pi = p;
# di = d;
pi = p2
di = d2

# load('templateSNS.mat')
mat3 = scipy.io.loadmat("templateSNS.mat")
p3 = mat3["p"]
d3 = mat3["d"]

# pt = p;
# dt = d;
pt = p3
dt = d3

# im = imread('rgb0001.jpg')
# im = imread('rgb0014.jpg')
im = imread("rgb0014.jpg")

# tm = imread('templateSNS.jpg')
tm = imread("templateSNS.jpg")

fig = plt.figure()

# figure 
ax1 = fig.add_subplot(121)

# subplot(121)
ax1.imshow(tm)

# imagesc(tm)
ax1.plot(pt[0,:], pt[1,:], "r.")

# hold on
# plot(pt(1,:), pt(2,:), 'r.')
ax2 = fig.add_subplot(122)

# subplot(122)
ax2.imshow(im)

# imagesc(im)
ax2.plot(pi[0,:], pi[1,:], "g.")

# plt.show()

# hold on
# plot(pi(1,:), pi(2,:), 'g.')

# calculate = 1; % 1 - homography; 2 - matches
calculate = 1

# switch calculate
if calculate == 1:
    # load('indices0001.mat')
    mat1 = scipy.io.loadmat("indices0001.mat")
    indices1 = mat1["indices"]

    # load('indices0014.mat')
    mat2 = scipy.io.loadmat("indices0014.mat")
    indices2 = mat2["indices"]

    # % indices_sorted = indices;
    # % [~,id] = unique(indices_sorted(:,2));
    # % indices_sorted = indices_sorted(id,:);

    # temp_points = [pt(1,indices(:,2)); pt(2,indices(:,2))];.
    print(indices1[:,2])
    temp_points = np.array([pt[0, indices1[:,2]], pt[1, indices1[:,2]]])

    # im_points = [pi(1,indices(:,1)); pi(2,indices(:,1))];
    im_points = np.array([pi[0, indices1[:,1]], pi[1, indices1[:,1]]])

    # tic;
    # [best_homography, im_points,temp_points] = RANSAC(im_points,temp_points, 0.5, 10000);
    best_homography, im_points, temp_points = RANSAC(im_points, temp_points, 0.5, 10000)

    # toc

    # random_indices = datasample(1:size(im_points,2),4);
    random_indices = np.random.choice(range(im_points.shape[1]), size=4, replace=False)

    # figure
    fig = plt.figure()

    # subplot(121);
    ax1 = fig.add_subplot(121)

    # imagesc(im);
    ax1.imshow(im)

    # hold on
    # plot(im_points(1,random_indices(1)), im_points(2,random_indices(1)), 'r.', 'MarkerSize',20)
    ax1.plot(im_points[0, random_indices[0]], im_points[1, random_indices[0]], "r.", markersize=20)

    # hold on
    # plot(im_points(1,random_indices(2)), im_points(2,random_indices(2)), 'g.', 'MarkerSize',20)
    ax1.plot(im_points[0, random_indices[1]], im_points[1, random_indices[1]], "g.", markersize=20)

    # hold on
    # plot(im_points(1,random_indices(3)), im_points(2,random_indices(3)), 'b.', 'MarkerSize',20)

    # hold on to the current figure and plot the first set of points in blue
plt.hold(True)
plt.plot(temp_points[0][random_indices[3]], temp_points[1][random_indices[3]], 'b.', markersize=20)

# plot the second set of points in yellow
plt.plot(temp_points[0][random_indices[4]], temp_points[1][random_indices[4]], 'y.', markersize=20)

# create a new figure and subplot
fig = plt.figure()
ax1 = fig.add_subplot(121)

# plot the first set of images in the first subplot
ax1.imshow(tm)
ax1.hold(True)
ax1.plot(temp_points[0], temp_points[1], 'r.')

# plot the second set of images in the second subplot
ax2 = fig.add_subplot(122)
ax2.imshow(im)
ax2.hold(True)
ax2.plot(im_points[0], im_points[1], 'g.')