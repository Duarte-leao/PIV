import numpy as np
import random

def RANSAC(im_points, temp_points, threshold, num_iter):
    best_inliers = [0, 0]
    for i in range(1, num_iter + 1):
        rand_ind = random.sample(range(0, len(im_points[0])), 4)
        H = homography(im_points[:, rand_ind], temp_points[:, rand_ind])
        inliers = [i for i in range(len(im_points[0])) if distance(im_points[:, i], temp_points[:, i], H) < 2]

        if len(inliers) > len(best_inliers):
            i
            best_inliers = inliers
            if len(best_inliers) > len(im_points[0]) * threshold:
                break

    im_points = im_points[:, best_inliers]
    temp_points = temp_points[:, best_inliers]
    best_homography = homography(im_points, temp_points)
    return best_homography, im_points, temp_points

def homography(im_points, temp_points):
    X = [im_points.T, np.ones((len(im_points[0]), 1))]
    M = np.zeros((2 * len(im_points[0]), 9))
    for i in range(len(im_points[0])):
        M[2*i-1:2*i+1, 0:3] = X[i]
        M[2*i-1:2*i+1, 4:7] = X[i]
        M[2*i-1:2*i+1, 7:10] = -X[i] * temp_points[:, i]
    U, S, V = np.linalg.svd(M.T.dot(M))
    H = np.reshape(V[:, -1], (3, 3)) / V[-1, -1]
    return H

def distance(p_im, p_temp, H):
    p_im = np.concatenate((p_im, [1]))
    p_temp = np.concatenate((p_temp, [1]))
    p_estimate = H.dot(p_im)
    p_estimate = p_estimate / p_estimate[-1]
    dist = np.linalg.norm(p_temp - p_estimate)
    return dist
