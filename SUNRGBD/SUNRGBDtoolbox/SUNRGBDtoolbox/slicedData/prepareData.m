%   load('/media/closerbibi/internal3/3D/understanding/rankpooling/rgbd_data/nyu_v2_labeled/nyu_depth_v2_labeled.mat'...
%   ,'depths','labels','names','accelData','instances')

addpath('~/3D/understanding/rankpooling/tsdf/');
addpath('~/bin/faster-rcnn/tools/')

count=1;
keySet=[];
valueSet=[];
mean1=0;mean2=0;mean3=0;mean4=0;
for imagenum = 1:1449
    tic;
    %% bed=157, chair=5, table=19, sofa=83, toilet=124
    try
        load(sprintf('../alignData/image%04d/annotation_pc.mat',imagenum)); % pc generate by ../seeAlignment_pc_3dBox.m
    catch
        continue
    end
    points3d = points3d';
	normalAndpcfile=sprintf('./normalAndpc/normalAndpc%06d.mat',imagenum);
	if ~exist(normalAndpcfile)
    	normals = depth2normal(points3d,1); % f unknown(normalize term), set it to '1'
		save(normalAndpcfile,'normals','points3d')
	else
		load(normalAndpcfile)
    end	
    pooling =0;
    [grid,mean_all] = slicedto2D(points3d, normals, imagenum, xmin, xmax, ymin, ymax,pooling);
    
    %% computing mean
    mean1 = mean1 + mean_all(1);
    mean2 = mean2 + mean_all(2);
    mean3 = mean3 + mean_all(3);
    mean4 = mean4 + mean_all(4);
    %%
    count=count+1;
	toc;
    disp(imagenum)
end    

%% computing mean
mean_all = [mean1 mean2 mean3 mean4]./count;
disp(mean_all)

%% generate box (already done)

%% run rankpooling
if pooling
    mean_all = vl_nnarpooltemporal(1449,13,1);
    disp(mean_all)
end    


% mapObj = containers.Map(keySet,valueSet);
% save('feature/imgIDmatch.mat','mapObj','keySet','valueSet')
