function vl_nnarpooltemporal(nScenes, layers)
% author: Hakan Bilen
% approximate rank pooling
% ids indicates frame-video association (must be in range [1-N])

%sz = size(X);
%forward = logical(nargin<3);
forward = logical(1);

%if numel(ids)~=size(X,4)
%  error('Error: ids dimension does not match with X!');
%end

%nScenes = 1449;%max(ids);

%if forward
%  grid = zeros([600,350,4,nScenes]);%,'like',X);
%else
%  grid = zeros(size(X),'like',X);
%end

%X=zeros([600,350,4,25]);
path_target='~/bin/faster-rcnn/data/DIRE/Images';
path_source='./picture_forPooling';
%path='~/bin/faster-rcnn/data/v_BlowDryHair_g12_c04_mp4';
for v=1:nScenes
  % pool among frames
  indv = 1:layers;%((v-1)*25+1):((v-1)*25+25);%find(ids==v);
  try
      checksize=load(sprintf('%s/picture_%06d_%03d.mat',path_source,v,1),'grid');
      v
  catch
      continue
  end
  imgsize = size(checksize.grid);
  X=zeros([imgsize,layers]);
  for count = 1:layers
	  tmp=load(sprintf('%s/picture_%06d_%03d.mat',path_source,v,count),'grid');
	  X(:,:,:,count)=tmp.grid;
  end
  if isempty(indv)
    error('Error: No frames in video %d',v);
  end
  N = numel(indv);
  % magic numbers
  fw = zeros(1,N);
  if N==1
    fw = 1;
  else
    for i=1:N
      fw(i) = sum((2*(i:N)-N-1) ./ (i:N));
    end
  end
  
  if forward
    grid =  sum(bsxfun(@times,X(:,:,:,indv),...
      reshape(single(fw),[1 1 1 numel(indv)])),4);    
  else
  %  Y(:,:,:,indv) = (bsxfun(@times,repmat(dzdy(:,:,:,v),[1,1,1,numel(indv)]),...
  %    reshape(fw,[1 1 1 numel(indv)])));
  end
  save(sprintf('%s/picture_%06d.mat',path_target,v),'grid');
end
%
% if forward
  %   fprintf(' fwd-arpool %.2f ',sqrt(sum(Y(:).^2)));
  % else
  %   fprintf(' back-arpool %f ',sqrt(sum(Y(:).^2)));
% end

