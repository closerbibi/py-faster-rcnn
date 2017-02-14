layers=25;
%chair=5,  table=19, sofa=83, toilet=124, bed=157
keyset = [5, 19, 83, 124, 157];
valset = {'chair', 'table', 'sofa', 'toilet', 'bed'};
mapObj = containers.Map(keyset,valset);

for imgXlayer=1:1449
    try
			load(sprintf('./feature/target_class_only/rankpooling/picture_forAnnotation/picture_%06d.mat',imgXlayer),'gridlabel2D','gridinstance2D');
    catch
        continue
    end
		fileID = fopen(sprintf('../../../../data/Annotations/picture_%06d.txt',imgXlayer),'w');

	%% creating bbox

    classNo = unique(gridlabel2D);
	target_class = find(classNo==157| classNo==5 |classNo==19 | classNo==83 |classNo==124);
    classNo = classNo(target_class); % eliminate unlabeled, 0 means unlabeled
    for class=1:length(classNo)
        for inst = 1:length(unique(gridinstance2D))-1
            [rowy,columnx] = find(gridlabel2D==classNo(class) & gridinstance2D==inst);
            if isempty(rowy) || isempty(columnx)
                continue
            end
            xmax = max(columnx);
            ymax = max(rowy);
            xmin = min(columnx);
            ymin = min(rowy);
            % minus one for python
            fprintf(fileID,'(%u, %u) - (%u, %u) - (%s)\n',xmin-1,ymin-1,xmax-1,ymax-1,mapObj(classNo(class)));%classNo(class));
        end    
    end
    fclose(fileID);
    imgXlayer
end
