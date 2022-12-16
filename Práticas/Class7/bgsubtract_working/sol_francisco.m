clear
clc
%% Stores data
%%Stores all files ended with .jpg
d=dir('*.jpg');
%%Stores all files ended with .mat
dd=dir('*.mat');
ims=[];
imsd=[];
%% Plays video
for i=1:length(d)
    imc=imread(d(i).name);
    im=rgb2gray(imc);
    figure(1)
    imshow(im); 
    colormap(gray);
    load(dd(i).name);
    drawnow;
    ims=[ims im(:)];
    imsd=[imsd depth_array(:)];
end

%% We could think of seeing the difference between final and beginning depth values
%initial
figure(2)
subplot(1,2,1)
imagesc(reshape(imsd(:,1), [480,640]))
title("Initial depth values")

%final
subplot(1,2,2)
imagesc(reshape(imsd(:,end), [480,640]))
title("Final depth values")

%difference
figure(3)
%subplot(2,2
forgmask = reshape((imsd(:,end)-imsd(:,1)),[480,640]);
imagesc(forgmask)
title("Difference from beggining depth values")

%% 
for i = 1:size(imsd,2)-1
    forgmask = reshape(abs(ims(:,i+1)-ims(:,i)),[480,640]);
    figure(4),imagesc(forgmask)
    pause(.5)
end

%%
figure(5)
subplot(1,2,1)
avg = reshape(mean(ims,2), [480,640]);
imagesc(avg), colormap gray
title("Mean")
hold on
avg = reshape(mean(ims,2), [480,640]);
imagesc(avg), colormap gray
subplot(1,2,2)
avg = reshape(median(ims,2), [480,640]);
imagesc(avg), colormap gray
title("Median")

%% MEDIAN IS THE WAY TO GO!
bgim = reshape(median(ims,2), [480,640]); 
%% find foreground (moving parts)

for i = 1:size(ims,2)
    forgmask = abs(reshape(ims(:,i),[480,640])-bgim);
    figure(6)
    imagesc(forgmask)
    pause(.5)
end


%% Difference bigger than 5
for i = 1:size(ims,2)
    forgmask = abs(reshape(ims(:,i),[480,640])-bgim)>5;
    figure(7)
    imagesc(forgmask)
    pause(.5)
end

%% Apply an operator that melhora a imagem (pesquisar sobre "erosion and opening")
for i = 1:size(ims,2)
    forgmask = imopen(abs(reshape(ims(:,i),[480,640])-bgim)>10, strel('disk',3));
    figure(8)
    imagesc(forgmask)
    pause(.5)
end


%% Apply an operator that melhora a imagem (pesquisar sobre "erosion and opening")
for i = 1:size(ims,2)
    forgmask = imopen(abs(reshape(ims(:,i),[480,640])-bgim)>10, strel('disk',3));
    foreg = double(foregmask).*double(reshape(ims(:,1),[480,640]));
    figure(8)
    imagesc(forgmask)
    pause(.5)
end


