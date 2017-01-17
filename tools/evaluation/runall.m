target_classes = {'chair','table','sofa','bed','toilet'}
for kkk = 1:5
    target_class = target_classes{kkk};
    % test
    %fpath = sprintf('../../results/test/comp4-27463_det_test_%s.txt',target_class);
    % train
    fpath = sprintf('../../results/train/comp4-19189_det_train_%s.txt',target_class);
    label_parent_dir = '../../data/DIRE/Annotations';
    gt_file_list = dir(label_parent_dir);
    [bboxes, confidences, image_ids] = fetch_result(fpath);

    unique_image = unique(image_ids);

    all_tp=0; all_fp=0; all_box_num = 0; all_gt_box_num =0;
    for i = 1:length(unique_image)
        ids = find(image_ids==unique_image(i));
        label_path = fullfile(label_parent_dir,sprintf('picture_%06d.txt',unique_image(i)));
        [gt_ids, gt_bboxes, gt_isclaimed, tp, fp, duplicate_detections] = ...
            evaluate_detections(bboxes(ids,:), confidences(ids,:), image_ids(ids,:), label_path, 0, target_class);
        con_idx=find(confidences(ids,:)>0.9);
        if ~isempty(tp)
            all_tp = all_tp + sum(tp(con_idx));
            all_fp = all_fp + sum(fp(con_idx));
            all_box_num = all_box_num + length(tp(con_idx));
            all_gt_box_num = all_gt_box_num + size(gt_ids,1);
        end
    end
    disp(target_class)
    precision=all_tp/all_box_num;
    disp(sprintf('precision: %d/%d = %.02f%% \n',all_tp,all_box_num,precision*100));

    recall=all_tp/all_gt_box_num;
    disp(sprintf('recall: %d/%d = %.02f%% \n',all_tp,all_gt_box_num,recall*100));
end
