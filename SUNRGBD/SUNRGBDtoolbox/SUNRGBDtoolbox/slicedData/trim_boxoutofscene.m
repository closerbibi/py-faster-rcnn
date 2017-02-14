function [xmin,xmax,ymin,ymax] = trim_boxoutofscene(xmin,xmax,ymin,ymax,width,height)
for i = 1:length(xmin)
    if xmin(i) < 0;      xmin(i) = 1;end
    if ymin(i) < 0;      ymin(i) = 1;end
    if xmin(i) > width;  xmin(i) = width-1;end
    if ymin(i) > height; ymin(i) = height-1;end    
    
    if xmax(i) < 0;      xmax(i) = 1;end
    if ymax(i) < 0;      ymax(i) = 1;end    
    if xmax(i) > width;  xmax(i) = width-1;end
    if ymax(i) > height; ymax(i) = height-1;end
end
    
