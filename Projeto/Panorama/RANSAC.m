function [best_homography, im_points,temp_points] = RANSAC(im_points,temp_points, threshold, num_iter)
    best_inliers = [0 0];
    for i = 1:num_iter
        rand_ind = datasample(1:size(im_points,2),4);
        H = homography(im_points(:,rand_ind), temp_points(:,rand_ind));
        inliers = find(distance(im_points, temp_points, H)<5);

        if size(inliers,2) > size(best_inliers,2)
            i
            best_inliers = inliers;
            if size(best_inliers,2) > size(im_points,2)*threshold
               break 
            end
        end
    end
    im_points = im_points(:,best_inliers);
    temp_points = temp_points(:,best_inliers);
    best_homography = homography(im_points, temp_points);
end
% ver porque é que nao tenho sempre um mínimo de 4 inliers
function H = homography(im_points,temp_points)
    X = [im_points',ones(size(im_points',1),1)];
    M = zeros(2*size(im_points',1),9);
    for i = 1:size(im_points',1)
        M(2*i-1,1:3) = X(i,:);
        M(2*i,4:6) = X(i,:);
        M(2*i-1,7:9) = X(i,:);
        M(2*i,7:9) = X(i,:);
    end
    M(:,7:9) = M(:,7:9).*-repmat(temp_points(:), 1, 3);
    [~,~,V] = svd(M'*M);
    H = reshape(V(:,end),[3 3])'/V(end,end);
 
end

function dist = distance(p_im, p_temp, H)
    p_im = [p_im;ones(1,size(p_im,2))];
    p_temp = [p_temp;ones(1,size(p_im,2))];
    p_estimate = H*p_im;
    p_estimate = p_estimate./p_estimate(3,:);
    dist = vecnorm(p_temp-p_estimate);
end