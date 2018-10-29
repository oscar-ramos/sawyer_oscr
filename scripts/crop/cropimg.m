
if (false)
    folder = '~/Desktop/';
    number = 6;
    I = imread([folder num2str(number) '.png']);
    Io = I(100:600, 650:1100, :);
    outname = ['out_' num2str(number) '.png']
    imwrite(Io, [folder outname])
    imshow(Io)
end

folder = '/tmp/';
maxnum = 16;
value = zeros(1,1,3)
for k=1:maxnum
    I = imread([folder num2str(k) '.png']);
    % imshow(I)
    Io = I(120:600, 560:950, :);
    outname = ['out_' num2str(k) '.png']
    imwrite(Io, [folder outname])
    %imshow(Io)
end
