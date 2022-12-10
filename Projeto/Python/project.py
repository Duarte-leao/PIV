import numpy as np
import scipy.io
import matplotlib.pyplot as plt
import math
import random


####### Functions #######

def NNeighbour_slow(dt,di):

    dt = dt.T
    di = di.T
    
    n_matches = min(dt.shape[0],di.shape[0]) #We will match all the key_points of the 'smaller' image
    poss_matches = max(dt.shape[0],di.shape[0])
    matches_aux = (0,0)
    matches = []

    for i2 in range(n_matches):
        distance = 10000
        for i1 in range(poss_matches):
            new_distance = math.dist(di[i2],dt[i1])
            if new_distance < distance: 
                distance = new_distance
                matches_aux = (i2,i1)
        matches = matches_aux
        print('iteration:',i2)    
    print(matches)
    return 
     
def NNeighbour(dt,di):
    
    n_matches = min(d1.shape[1],d2.shape[1])
    matches = np.zeros((n_matches,2))
    matches[:,0] = np.arange(0,n_matches)

    for i in range(n_matches):
        dist = np.linalg.norm(di[:,i] - dt.T, axis = 1)
        matches[i,1] = np.argmin(dist)
    
    return matches

################## 1 - Extract Keypoints using SIFT ##################

#Extract coordinates
img2 = plt.imread('rgb0001.jpg')
template = plt.imread('templateSNS.jpg')

#Extract descriptors and keypoints
img2_m = scipy.io.loadmat('rgb0001.mat')
template_m = scipy.io.loadmat('templateSNS.mat')

#descriptors: d
d2 = img2_m['d']
d1 = template_m['d']

#key points:
p2 = img2_m['p']
p1 = template_m['p']

################## 2 - Match the keypoints between template and image ##################

# How do we do this ?
# Nearest neighbour between the descriptors of the template and image ? Yes
# If I am not mistaken descriptors have like gradients inside them

####### Implementation #######
# The implementation will be done by brute force
# For each descriptor in Image 2 we will search all the descriptors in the Template to find which as the minimum
# Distance 

p2 = p2.T
p1 = p1.T

matching = NNeighbour(d1,d2)
print(matching)



''' Sanity check '''
sanity = []
for i in range(4):
    sanity.append(matching[i + random.randint(100,4000)])
sanity = np.array(sanity)

fig, axes = plt.subplots(nrows=1, ncols=2, figsize=(5, 3))
axes[0].imshow(img2)
axes[1].imshow(template)
#axes[0].scatter(p2[:,0] , p2[:,1], s = 1)
for i in range(4):
    axes[0].scatter(p2[int(sanity[i,0]),0] , p2[int(sanity[i,0]),1], s = 1)
    axes[1].scatter(p1[int(sanity[i,1]),0],p1[int(sanity[i,1]),1], s = 1)   
plt.show()

################## 3 - RANSAC ##################

# 1) Randomly select n samples (n << N)

'''
(i) We have four variables, this means that we will have n = 4
(ii) But why do we have four variables ?
'''


# 2) Finding the model that fits this n points [Fit]

'''
(i) Here we will compute the Homography 
(ii) Homography will be the the solution of a Least Squares problem: Ax = 0
(iii) A will be our 8 points (4 pairs of points)
(iiii) x is our variable, the homography
'''

# 3) Determing inliners [Predict]

'''
(i) Here we have to define a threshold (m) that will tell us which points
are inliers and which points aren't
'''

# 4) Select the best model, i.e., the one that contains the most inliers

