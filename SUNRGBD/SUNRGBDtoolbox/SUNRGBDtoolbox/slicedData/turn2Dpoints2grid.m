function [gridindex,new_xmin,new_xmax,new_ymin,new_ymax]=turn2Dpoints2grid(points2D,xmin,xmax,ymin,ymax)
%movx=points2D(1,1:5:end)-min(points2D(1,1:5:end));
%movy=points2D(2,1:5:end)-min(points2D(2,1:5:end));
movx=points2D(1,:)-min(points2D(1,:));
tmp_xmin=xmin-min(points2D(1,:));
tmp_xmax=xmax-min(points2D(1,:));
movy=points2D(2,:)-min(points2D(2,:));
tmp_ymin=ymin-min(points2D(2,:));
tmp_ymax=ymax-min(points2D(2,:));


points2D100x=round(movx.*100)+1;
new_xmin=round(tmp_xmin.*100)+1;
new_xmax=round(tmp_xmax.*100)+1;
points2D100y=round(movy.*100)+1;
new_ymin=round(tmp_ymin.*100)+1;
new_ymax=round(tmp_ymax.*100)+1;

%points2D100x=round(movx.*((width-1)/max(movx)))+1;
%points2D100y=round(movy.*((height-1)/max(movy)))+1;
gridindex = [points2D100x;points2D100y];

% max(show100x)
% min(show100x)
% max(show100y)
% min(show100y)
% 
% scatter(show100x,show100y)
