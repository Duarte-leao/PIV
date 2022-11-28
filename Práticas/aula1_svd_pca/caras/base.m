%%
%p is the location of eyes and mouth to crop images around the face
%load it from loadme_parameters.mat (all data)
load loadme_parameters.mat
%To register face images (eyes&mouth) run this:
%d=dir('*.jpg');
%p={};for i=1:length(d),imagesc(imread(d(i).name));p=[p {ginput(3)}];end
%pb=p{3};
%ims=[];
%d=dir('*.jpg');
%%
for i=1:length(d),
    im=rgb2gray(imread(d(i).name));
    im=im(round(p{i}(1,2))-60:round(p{i}(1,2))+80,round(p{i}(1,1))-30:round(p{i}(1,1))+80);
    figure(1);imagesc(im);colormap(gray);pause;
    drawnow;
    ims=[ims im(:)];
end
ims=double(ims);
caramedia=mean(ims')';
imsc=imsc-caramedia*ones(1,size(ims,2));
imagesc(reshape(caramedia,141,111));pause(2);
%%
%for i=13:-1:1,imagesc(reshape(imsc(:,i),141,111));pause,end
%[v,s]=eig(imsc'*imsc);
%u=imsc*v*inv(sqrt(s));
[u s v]=svd(imsc,'econ');
eigfaces=[];
for i=1:12,
    eigfaces=[eigfaces reshape(u(:,i),141,111)];
end
%%
for i=13:-1:1,imagesc(reshape(u(:,i),141,111));pause,end
%%
%LOAD ME AND RUN ME
im1=ims(:,1);
im1c=imsc(:,1);
aux=[reshape(im1,141,111),reshape(caramedia,141,111)];
figure(1);imagesc(aux);colormap(gray);
figure(3);subplot(2,1,1);imagesc(eigfaces);colormap(gray);
for i=1:12,
    figure(2);
    imagesc(reshape(u(:,1:i)*u(:,1:i)'*im1c+caramedia,141,111));
    colormap(gray);
    figure(3);subplot(2,1,2);
    plot(u(:,1:i)'*im1c);ax=axis;ax(1)=.5;ax(2)=12.5;axis(ax);
    fprintf('component %d - type any key \n', i);
    pause,
end




