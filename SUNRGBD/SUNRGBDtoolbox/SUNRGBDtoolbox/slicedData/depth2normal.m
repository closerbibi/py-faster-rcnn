function normals  = depth2normal(XYZcam,f)

points3D = ones(4,length(XYZcam));
points3D(1:3,:) = XYZcam;

width  = 480;
height = 640;

[viewMap(:,:,1) viewMap(:,:,2)] = meshgrid(1:width,1:height);
viewMap(:,:,1) = viewMap(:,:,1) - width/2 - 0.5;
viewMap(:,:,2) = viewMap(:,:,2) - height/2 - 0.5;
viewMap(:,:,3) = f;
viewMap = viewMap/f;
viewVectors = reshape(viewMap,[],3)';

normals = points2normals(points3D(1:3,:));

% dotp = sum(normals .* viewVectors,1)>0; % keep those point that is visible
% normals(:,dotp) = - normals(:,dotp);
