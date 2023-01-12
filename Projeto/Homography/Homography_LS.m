%function [final_homography, im_points,temp_points] = Homography_LS(best_homography, im_points,temp_points)
function final_homo = Homography_LS(best_homography, im_points,temp_points)
        
    options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt');
    options.StepTolerance = 1.000000000000000e-20;
    %options = optimoptions(@lsqnonlin,'Algorithm','trust-region-reflective','SpecifyObjectiveGradient',true);
    [final_homo,~,~,~,~] = lsqnonlin(@(x)cost_function(x, im_points, temp_points), best_homography,[],[],options);
end
