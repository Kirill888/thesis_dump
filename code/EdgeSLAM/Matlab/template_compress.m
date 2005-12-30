function [t1,t2] = template_compress(ll,do_plot)
%function [t1,t2] = template_compress(ll,do_plot)

global LEFT_MASK 
global RIGHT_MASK

if nargin < 2
  do_plot = 0;
end

fr = ll(1,1);
to = ll(end,1);

if do_plot
  fig1 = figure(1);
  fig2 = figure(2);
end

W = [-3:3];
t1 = zeros(size(ll,1),length(W)*3);
if nargout > 1
  t2 = zeros(size(ll,1),length(W)*3);
end

n_t = 0;

for i = fr:to
  disp(sprintf('Loading Images: %d',i));
  
  I1 = imread(sprintf(LEFT_MASK,i));
  
  if nargout > 1
    I2 = imread(sprintf(RIGHT_MASK,i));
  end

  ind = find(ll(:,1) == i);
  l1 = ll(ind,2);
  l2 = ll(ind,3);
  h  = ll(ind,5:6);

  if do_plot
    figure(fig1);
    cla;
    imshow([I1,I2]); hold on;

    line([l1,l1]',h','LineWidth',2);
    line(640 + [l2,l2]', h','LineWidth',2);
  end
  
  for j = 1:size(l1,1)
    n_t = n_t + 1;
    
    T1 = I1(h(j,1):h(j,2), W + l1(j),:);
    T1_c = compress(T1);
    t1(n_t,:) = T1_c(:)';
    
    if nargout > 1
      T2 = I2(h(j,1):h(j,2), W + l2(j),:);
      T2_c = compress(T2);
      t2(n_t,:) = T2_c(:)';
    end
    
    if do_plot
      figure(fig2); 
      subplot(1,4,1);
      imshow(T1);
      subplot(1,4,2);
      plot(T1_c');
      axis([1, 7, -0.5, 0.5]);
      subplot(1,4,3);
      imshow(T2);
      subplot(1,4,4);
      plot(T2_c');
      axis([1, 7, -0.5, 0.5]);
      pause
    end
  end

  if do_plot
    pause;
  end
end

function T_c = compress(T)
  T = double(T);
  T_c = [mean(T(:,:,1)); mean(T(:,:,2)); mean(T(:,:,3))];

  %normalise
  mm = mean(T_c(:));
  T_c = T_c - mm;
  T_c = T_c/sqrt(T_c(:)'*T_c(:));

  
%  T = double(T);
%  T_c = [median(T(:,:,1)); median(T(:,:,2)); median(T(:,:,3))];
  