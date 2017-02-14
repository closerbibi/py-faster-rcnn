function [grid,mean_all] = slicedto2D(points3D,normals,count,xmin,xmax,ymin,ymax,pooling)
upright = [1e-06,1e-06, 1];
points_value = points3D'*upright';
d = max(points_value);
parts=29;
step_size = (max(points_value) - min(points_value))/parts;

%% debug
% figure(3);
% pt = mean(points3D,1);
% quiver3(pt(1),pt(2),pt(3),upright(1)*1,upright(2)*1,upright(3)*1,'r','linewidth',2);hold on;
% scatter3(points3D(1,1:50:end)',points3D(2,1:50:end)',points3D(3,1:50:end)')

points2D = points3D(1:2,:);
% augment data with rotation, faster-rcnn will flip left to right(or right to left whatever)
% rotate
times_rot = 1;
rt = (0/2)+1;
points2D_rot = zeros([size(points2D) rt]);
xmin_rot = zeros([size(xmin) rt]);xmax_rot = zeros([size(xmax) rt]);
ymin_rot = zeros([size(ymin) rt]);ymax_rot = zeros([size(ymax) rt]);

for rot = 1:rt
    theta = 2*pi*(rot-1)/times_rot;
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    points2D_rot(:,:,rot) = R*points2D;
    for kr = 1:size(xmax,2)
        p1 = R*[xmin(kr) xmax(kr);ymin(kr) ymax(kr)];
        xmin_rot(:,kr,rot) = min(p1(1,1),p1(1,2));
        ymin_rot(:,kr,rot) = min(p1(2,1),p1(2,2));
        xmax_rot(:,kr,rot) = max(p1(1,1),p1(1,2));
        ymax_rot(:,kr,rot) = max(p1(2,1),p1(2,2));         
%         tmp1 = R*[xmin(kr);ymax(kr)];
%         tmp2 = R*[xmax(kr);ymin(kr)];
%         xmin_rot(:,kr,rot) = min(tmp1(1),tmp2(1));
%         ymin_rot(:,kr,rot) = min(tmp1(2),tmp2(2));
%         xmax_rot(:,kr,rot) = max(tmp1(1),tmp2(1));
%         ymax_rot(:,kr,rot) = max(tmp1(2),tmp2(2)); % min max might have more than one set, deal with it carefully
    end   
    
    %% normalize into a grid
    [gridindex_rot,xmin_rot(:,:,rot),xmax_rot(:,:,rot),ymin_rot(:,:,rot),ymax_rot(:,:,rot)] = ...
        turn2Dpoints2grid(points2D_rot(:,:,rot),xmin_rot(:,:,rot),xmax_rot(:,:,rot),ymin_rot(:,:,rot),ymax_rot(:,:,rot));
    width_rot = max(gridindex_rot(1,:));
    height_rot = max(gridindex_rot(2,:));
    %% trim the box out of range
    [xmin_rot(:,:,rot),xmax_rot(:,:,rot),ymin_rot(:,:,rot),ymax_rot(:,:,rot)] = ...
        trim_boxoutofscene(xmin_rot(:,:,rot),xmax_rot(:,:,rot),ymin_rot(:,:,rot),ymax_rot(:,:,rot),width_rot,height_rot);
    
    d = max(points_value);
    %% forPooling
    if pooling
        target_layers = 15:27;
        for k=1:parts%d > min(points_value)
            range_d = d-(k-1)*step_size;
            inZone = logical((points_value<range_d).*(points_value>(range_d-step_size)));
            normals2D = normals(:,inZone);
            gridindex_layer = gridindex_rot(:,inZone);
            grid = zeros(height_rot,width_rot,4);
            grid(sub2ind(size(grid),gridindex_layer(2,:),gridindex_layer(1,:)))=1;%points cloud 2D, might assign to the same grid cell
            grid(sub2ind(size(grid),gridindex_layer(2,:),gridindex_layer(1,:))+height_rot*width_rot)= normals2D(1,:); %normal x
            grid(sub2ind(size(grid),gridindex_layer(2,:),gridindex_layer(1,:))+height_rot*width_rot*2)= normals2D(2,:); %normal y
            grid(sub2ind(size(grid),gridindex_layer(2,:),gridindex_layer(1,:))+height_rot*width_rot*3)= normals2D(3,:); %normal z

            if any(k==target_layers)
                save(sprintf('./picture_forPooling/picture_%06d_%03d.mat',count,k-target_layers(1)+1),'grid');
            end
    %         find(inZone~=0) %% debug
        end
    end
    
    %% projecting
    if ~pooling
        target_layers = 3:27; % from top to bottom

        range_upper = d-(target_layers(1)-1)*step_size; 
        range_lower = d-(target_layers(end)-1)*step_size;
        inZone = logical((points_value<range_upper).*(points_value>range_lower));
        normals2D = normals(:,inZone);
        gridindex_layer = gridindex_rot(:,inZone);
        grid = zeros(height_rot,width_rot,4);
        grid(sub2ind(size(grid),gridindex_layer(2,:),gridindex_layer(1,:)))=1;%points cloud 2D, might assign to the same grid cell
        grid(sub2ind(size(grid),gridindex_layer(2,:),gridindex_layer(1,:))+height_rot*width_rot)= normals2D(1,:); %normal x
        grid(sub2ind(size(grid),gridindex_layer(2,:),gridindex_layer(1,:))+height_rot*width_rot*2)= normals2D(2,:); %normal y
        grid(sub2ind(size(grid),gridindex_layer(2,:),gridindex_layer(1,:))+height_rot*width_rot*3)= normals2D(3,:); %normal z

        save(sprintf('./picture_readytotrain/picture_%06d.mat',count),'grid');
    end    

    %% computing mean
    mean1=mean(mean(grid(:,:,1)));
    mean2=mean(mean(grid(:,:,2)));
    mean3=mean(mean(grid(:,:,3)));
    mean4=mean(mean(grid(:,:,4)));
    mean_all = [mean1,mean2,mean3,mean4];
    
    %% save whole img in 2D
    grid_whole_img = zeros(height_rot,width_rot,1);
    grid_whole_img(sub2ind(size(grid),gridindex_rot(2,:),gridindex_rot(1,:))) = 1;
    save(sprintf('./picture_forPooling/bv_%06d.mat',count),'grid_whole_img');

    
    
    %{
    %% save ground truth annotation
    annofile = sprintf('~/bin/faster-rcnn/data/DIRE/Annotations/picture_%06d.txt',count);
    AnnotationID = fopen(annofile,'w');
    for box_l = 1:length(xmin)
        fprintf(AnnotationID,'(%u, %u) - (%u, %u) - (%s)\n',xmin_rot(:,box_l,rot)-1,ymin_rot(:,box_l,rot)-1,xmax_rot(:,box_l,rot)-1,ymax_rot(:,box_l,rot)-1,clss{box_l});
    end
    fclose(AnnotationID);
    %}



    %% debug
    %{
     figure(1)
%      imshow(~grid_whole_img)
     imshow(~grid(:,:,1))    
     axis on
     for showthis = 1:size(xmax,2)
         xi = xmin_rot(:,showthis,rot);
         yi = ymin_rot(:,showthis,rot);
         xa = xmax_rot(:,showthis,rot);
         ya = ymax_rot(:,showthis,rot);
         rectangle('Position',[xi yi (xa-xi) (ya-yi)],'EdgeColor','b','LineWidth',3);hold on;
     end
     close all
    %}
end


% % % % % %% save ground truth annotation
% % % % % annofile = sprintf('~/bin/faster-rcnn/data/DIRE/Annotations/picture_%06d.txt',count);
% % % % % AnnotationID = fopen(annofile,'w');
% % % % % for box_l = 1:length(xmin)
% % % % %     fprintf(AnnotationID,'(%u, %u) - (%u, %u) - (%s)\n',xmin(box_l)-1,ymin(box_l)-1,xmax(box_l)-1,ymax(box_l)-1,clss{box_l});
% % % % % end
% % % % % fclose(AnnotationID);


% % % % % %% forPooling
% % % % % for k=1:parts%d > min(points_value)
% % % % %     inZone = logical((points_value<d).*(points_value>(d-step_size)));
% % % % % 
% % % % %     normals2D = normals(:,inZone);
% % % % % 
% % % % %     
% % % % %     gridindex_layer = gridindex_rot(:,inZone);
% % % % %     grid = zeros(height_rot,width_rot,4);
% % % % %     grid(sub2ind(size(grid),gridindex_layer(2,:),gridindex_layer(1,:)))=1;%points cloud 2D
% % % % %     grid(sub2ind(size(grid),gridindex_layer(2,:),gridindex_layer(1,:))+height_rot*width_rot)= normals2D(1,:); %normal x
% % % % %     grid(sub2ind(size(grid),gridindex_layer(2,:),gridindex_layer(1,:))+height_rot*width_rot*2)= normals2D(2,:); %normal y
% % % % %     grid(sub2ind(size(grid),gridindex_layer(2,:),gridindex_layer(1,:))+height_rot*width_rot*3)= normals2D(3,:); %normal z
% % % % % 
% % % % % % % %     save(sprintf('./picture_forPooling/picture_%06d_%03d.mat',count,k),'grid');
% % % % %     d = d-step_size;
% % % % % end
% % % % % 
% % % % % grid_whole_img = zeros(height_rot,width_rot,1);
% % % % % grid_whole_img(sub2ind(size(grid),gridindex_rot(2,:),gridindex_rot(1,:))) = 1;
% % % % % % imshow(grid_whole_img);
% % % % % % % % save(sprintf('./picture_forPooling/picture_%06d.mat',count),'grid_whole_img');

