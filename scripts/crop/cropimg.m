
folder = '~/Desktop/';
number = 6;
I = imread([folder num2str(number) '.png']);
Io = I(100:600, 650:1100, :);
outname = ['out_' num2str(number) '.png']
imwrite(Io, [folder outname])
%imshow(Io)
