%%
%  IMAGE AS MATRIX, vector, pdf
figure(1);
im=double(imread('mms.jpg'));
imagesc(im/255);
figure(2);
%The RGB image
imagesc([im(:,:,1) im(:,:,2) im(:,:,3)]);
colormap(gray)
ims=rgb2gray(im/255);
figure(3);
%Image as a function
mesh(ims(1:2:end,1:2:end));axis ij;
% Compute gradient
[dx,dy]=gradient(ims);
[x,y]=meshgrid(1:8:size(ims,2),1:8:size(ims,1));
figure(4)
quiver(x,y,5*dx(1:8:end,1:8:end),5*dy(1:8:end,1:8:end ))
%%
% Linear Spaces SVD
im=imread('lena.gif');
imagesc(im);colormap(gray);
im=double(im);
[u,s,v]=svd(im);
%% compare least squares with total least squares(SVD)
r=randperm(256);
for i=1:length(s)-1,
    imr=zeros(size(im));
    k=i;
    sc=max(im(:));    
    imr=u(:,1:k)*s(1:k,1:k)*v(:,1:k)';
    b=im(:,r(1:i));
    %projector
    imr2=b*inv(b'*b)*b'*im;
    imagesc([im, imr imr2]);
    text(10,10,int2str(k));  
    drawnow;
    pause;
end
%%
imo=double(imread('voldemort.jpg'))/255;
imagesc(imo);
%imo=rgb2hsv(imo);
im=[imo(:,:,1);imo(:,:,2);imo(:,:,3)];
[u,s,v]=svd(im);
figure(2);imagesc(hsv2rgb(imo));
%% in color
r=randperm(256);
for i=1:length(s),
    imr=zeros(size(im));
    k=i;
    imr=u(:,1:k)*s(1:k,1:k)*v(:,1:k)';
    b=im(:,r(1:i));
    imr2=b*inv(b'*b)*b'*im;
    imd=imrotate((reshape(imr',[size(imo,2) size(imo,1) 3])),-90);
    imd2=imrotate((reshape(imr2',[size(imo,2) size(imo,1) 3])),-90);
    
    %imd=imrotate(hsv2rgb(reshape(imr',[size(imo,2) size(imo,1) 3])),-90);
    %imd2=imrotate(hsv2rgb(reshape(imr2',[size(imo,2) size(imo,1) 3])),-90);
    imagesc([(imo), imd imd2]);
    text(10,10,int2str(k));  
    drawnow;
    pause;
end
