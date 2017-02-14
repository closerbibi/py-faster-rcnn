function XYZcamera = depth2XYZcamera(K, depth)
    [x,y] = meshgrid(1:640, 1:480);
    XYZcamera(:,:,1) = (x-K(1,3)).*depth/K(1,1); % convert to the real distance with the central in the world
    XYZcamera(:,:,2) = (y-K(2,3)).*depth/K(2,2);
    XYZcamera(:,:,3) = depth;
    XYZcamera(:,:,4) = depth~=0; % ~= equal to !=, if depth info. valid showing '1'
end