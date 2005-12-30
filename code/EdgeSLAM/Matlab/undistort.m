function undistort(image_base,KK,kc)
% function undistort(image_base,KK,kc)
%
%  Undistort sequence of images.
%   image_base -- image  names with wildcards eg: 'left_*.pnm'
%   KK         -- internal camera matrix
%   kc         -- distortion coefficents
%

images = dir( image_base );

if isempty(images) 
  disp(['No images found:' image_base]);
  return;
end

I = imread(images(1).name);
[ny,nx,nc] = size(I);

disp(sprintf('Found %d images %dx%d',length(images),nx,ny));

fc = KK([1;5])
cc = KK(1:2,3)
alpha_c = KK(1,2)/fc(1)

% Pre-compute the necessary indices and blending coefficients:

[Irec_junk,ind_new,ind_1,ind_2,ind_3,ind_4,a1,a2,a3,a4] = ...
           rect_index(zeros(ny,nx),eye(3),fc,cc,kc,alpha_c,KK);

I0 = zeros(ny,nx);
I2 = zeros(ny,nx,nc);

for kk = 1:length(images)

  ima_name = images(kk).name;
  ima_name2 = ['undist_' ima_name];

  ima_name2(end-2:end) = 'ppm';
  if exist(ima_name2,'file') 
    disp(['Skipping:' ima_name]);
  
  else
    
    fprintf(1,'Loading original image %s...',ima_name);
    I = double(imread(ima_name));
  
    for ii = 1:nc
      
      Iii = I(:,:,ii);
      I2ii = I0;

%      I2ii(ind_new) = uint8(a1 .* Iii(ind_1) + a2 .* Iii(ind_2) + a3 .* Iii(ind_3) + a4 .* Iii(ind_4));
      I2ii(ind_new) = a1 .* Iii(ind_1) + a2 .* Iii(ind_2) + ...
	              a3 .* Iii(ind_3) + a4 .* Iii(ind_4);
      
      I2(:,:,ii) = I2ii;
      
    end;
    
    fprintf(1,'Saving undistorted image under %s...\n',ima_name2);
    imwrite(uint8(I2),ima_name2,'ppm');
  end
  
end


