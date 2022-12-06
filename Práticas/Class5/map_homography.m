%%
parede1=imread('parede1.jpg');
parede2=imread('parede2.jpg');

imshow(parede1);
title('Clicar nos quatro cantos do cartaz');
[u1 v1]=ginput(4);
figure;
imshow(parede2);
title('Clicar nos mesmo quatro cantos do cartaz e pela mesma ordem');
[u2 v2]=ginput(4);

pts=[0 0; 200 0; 200 300; 0 300];
t1=cp2tform([u1 v1],pts,'projective');
t2=cp2tform([u2 v2],pts,'projective');

im1=imtransform(parede1,t1,'Xdata',[-400 800],'YData',[-250 600]);
im2=imtransform(parede2,t2,'Xdata',[-400 800],'YData',[-250 600]);

close all hidden;

figure;
imshow(im1);
title('Imagem 1 mapeada nas coordenadas "world" do cartaz');
figure;
imshow(im2);
title('Imagem 2 mapeada nas coordenadas "world" do cartaz');
figure;
imagesc(uint8(double(im1)*.5+double(im2)*.5))
title('Ambas as imagens mapeadas nas coordenadas "world" do cartaz');
