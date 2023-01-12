close all
% clear

%% Code
%load('input/rgb0001.mat')
load('input/rgb0014.mat')
pi = p;
di = d;
load('template/templateSNS.mat')
pt = p;
dt = d;
%im = imread('input/rgb0001.jpg');
im = imread('input/rgb0014.jpg');
tm = imread('template/templateSNS.jpg');
figure 
subplot(121);
imagesc(tm);
hold on
plot(pt(1,:), pt(2,:), 'r.')
subplot(122);
imagesc(im);
hold on
plot(pi(1,:), pi(2,:), 'g.')

calculate = 1; % 1 - homography; 2 - matches

switch calculate
    case 1
        %load('indices0001.mat')
        load('indices0014.mat')
        
        % indices_sorted = indices;

        % [~,id] = unique(indices_sorted(:,2));

        % indices_sorted = indices_sorted(id,:);

        temp_points = [pt(1,indices(:,2)); pt(2,indices(:,2))];
        im_points = [pi(1,indices(:,1)); pi(2,indices(:,1))];

        tic;
        [best_homography, im_points,temp_points] = RANSAC(im_points,temp_points, 0.5, 10000);
        toc
        
        tic;
        best_homography_1 = LM(best_homography, im_points, temp_points);
        toc


        random_indices = datasample(1:size(im_points,2),4);
        figure
        subplot(121);
        imagesc(im);
        hold on
        plot(im_points(1,random_indices(1)), im_points(2,random_indices(1)), 'r.', 'MarkerSize',20)
        hold on
        plot(im_points(1,random_indices(2)), im_points(2,random_indices(2)), 'g.', 'MarkerSize',20)
        hold on
        plot(im_points(1,random_indices(3)), im_points(2,random_indices(3)), 'b.', 'MarkerSize',20)
        hold on
        plot(im_points(1,random_indices(4)), im_points(2,random_indices(4)), 'y.', 'MarkerSize',20)
        subplot(122);
        imagesc(tm);
        hold on
        plot(temp_points(1,random_indices(1)), temp_points(2,random_indices(1)), 'r.', 'MarkerSize',20)
        hold on
        plot(temp_points(1,random_indices(2)), temp_points(2,random_indices(2)), 'g.', 'MarkerSize',20)
        hold on
        plot(temp_points(1,random_indices(3)), temp_points(2,random_indices(3)), 'b.', 'MarkerSize',20)
        hold on
        plot(temp_points(1,random_indices(4)), temp_points(2,random_indices(4)), 'y.', 'MarkerSize',20)

        figure 
        subplot(121);
        imagesc(tm);
        hold on
        plot(temp_points(1,:), temp_points(2,:), 'r.')
        subplot(122);
        imagesc(im);
        hold on
        plot(im_points(1,:), im_points(2,:), 'g.')
        
        
        
        imOut = imwarp(im,projective2d(best_homography'));
        figure
        imshow(imOut)
        
        imOut_1 = imwarp(im,projective2d(best_homography_1'));
        figure
        imshow(imOut_1)
        
    case 2
        indices = zeros(size(di,2), 3);
        indices(:,1) = [1:size(di,2)]';
        tic;
        for i = 1:size(di,2)
            [indices(i,2), indices(i,3)] = NNeighbour(di(:,i), dt);
        end
        save('indices0001.mat','indices')
        save('indices0014.mat','indices')
        toc
end

function [index_of_matching_p, min_norm] = NNeighbour(di_point, dt)
    norm = vecnorm(di_point-dt);
    min_norm = min(norm);
    index_of_matching_p = find(norm == min(norm));
    index_of_matching_p =  datasample(index_of_matching_p,1);
end

function H_refined = LM(H, img_points, temp_points)
    % Define the objective function
    f = @(H, x1, x2) reshape(x2 - H*x1, [], 1);

    % Define the Jacobian of the objective function
    J = @(H, x1) -kron(x1', eye(3));

    % Define the initial damping factor (lambda)
    lambda = 0.001;

    % Define the termination tolerance
    tol = 1e-6;

    % Define the maximum number of iterations
    max_iter = 500;

    % Initialize the iteration counter
    iter = 0;

    % Initialize the homography matrix
    H_refined = H;

    % Convert the matched points to homogeneous coordinates
    %img_points = [img_points, ones(size(img_points,1),1)];
    %temp_points = [temp_points, ones(size(temp_points,1),1)];
    img_points = [img_points; ones(1,size(img_points,2))];
    temp_points = [temp_points; ones(1,size(temp_points,2))];

    % Compute the initial residual error
    residual_error = f(H_refined, img_points, temp_points)
    
    
    % Init residual error
    residual_error_prev = residual_error;

    while (iter < max_iter) && (norm(residual_error) > tol)
        % Compute the Jacobian matrix
        JH = J(H_refined, img_points);

        % Compute the Hessian matrix
        H_LM = JH'*JH + lambda*eye(size(JH,2));

        % Compute the update step
        delta_H = -H_LM\(JH'*residual_error);

        % Update the homography matrix
        H_refined = H_refined + reshape(delta_H,3,3);

        % Compute the new residual error
        residual_error = f(H_refined, img_points, temp_points);

        % Update the damping factor
        if (norm(residual_error) > norm(residual_error_prev))
            lambda = lambda*10;
        else
            lambda = lambda/10;
        end

        % Update the iteration counter
        iter = iter + 1;
    end
end