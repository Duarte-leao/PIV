function error = cost_function(best_homography, im_points, temp_points)

    points = [im_points; ones(1, size(im_points, 2))];
    proj_points = (best_homography * points);
    proj_points = proj_points(1:2,:)./proj_points(3,:);
    
    error = sum((proj_points - temp_points).^2);
    error = double(error);

end