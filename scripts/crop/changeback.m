folder = '/tmp/abc/str/tt/';
k = 16;
Io = imread([folder 'out_' num2str(k) '.png']);
% Remove background
for i=1:size(Io,1)
    for j=1:size(Io,2)
        if ((Io(i,j,1)==241) & (Io(i,j,2)==255) & (Io(i,j,3)==255))
            Io(i,j,1)=255; Io(i,j,2)=255; Io(i,j,3)=255;
        end
        if ((Io(i,j,1)==199) & (Io(i,j,2)==212) & (Io(i,j,3)==212))
            Io(i,j,1)=212; Io(i,j,2)=212; Io(i,j,3)=212;
        end
    end
end
imshow(Io)                           
outname = ['out2_' num2str(k) '.png']
imwrite(Io, [folder outname])
