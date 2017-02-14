for i =1:1449
try    
    figure(1)
    file1 = sprintf('picture_forPooling/picture_%06d_%03d.mat',i,1);
    load(file1)
    imshow(grid(:,:,1))
    title('layer 1')

    figure(2)
    file2 = sprintf('picture_forPooling/picture_%06d_%03d.mat',i,2);
    load(file2)
    imshow(grid(:,:,1))
    title('layer 2')

    figure(3)
    file24 = sprintf('picture_forPooling/picture_%06d_%03d.mat',i,12);
    load(file24)
    imshow(grid(:,:,1))
    title('layer 24')

    figure(4)
    file25 = sprintf('picture_forPooling/picture_%06d_%03d.mat',i,13);
    load(file25)
    imshow(grid(:,:,1))
    title('layer 25')

    figure(5)
    file = sprintf('../../../kv1/NYUdata/NYU%04d/image/NYU%04d.jpg',i,i);
    imshow(imread(file));
    title('image')
catch
    continue
end
pause
end

