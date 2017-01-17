gt_path = '/home/closerbibi/bin/faster-rcnn/data/DIRE/Annotations/'
resultpath = '/home/closerbibi/bin/faster-rcnn/results/train/comp4-9396_det_train_chair.txt'

% evaluate_detections(bboxes, confidences, image_ids, label_path, draw)

result_fid = fopen(resultpath);
reult_info = textscan(result_fid, '%s %d %d %d %d %d\n');
fclose(result_fid)
re

