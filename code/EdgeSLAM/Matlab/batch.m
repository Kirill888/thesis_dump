function out = batch(f_start, f_end, use_color)
% function out = batch(f_start, f_end, use_color)

global LEFT_MASK
global RIGHT_MASK

if nargin < 3
  use_color = 1;
end

nout = 0;

out = zeros(10000, 10);

global I1 I2

for i = f_start:f_end
  disp(sprintf('Loading Images: %d',i));

  if use_color
    I1 = imread(sprintf( LEFT_MASK, i));
    I2 = imread(sprintf(RIGHT_MASK, i));
  else
    I1 = rgb2gray(imread(sprintf(LEFT_MASK,i)));
    I2 = rgb2gray(imread(sprintf(RIGHT_MASK,i)));
  end
  [l1,l2,l2_w] = matchImages(0);
  
  n = size(l1,1);
  
  ind = (nout+1):(nout+n);
  out(ind,2:end) = [l1,l2,l2_w];
  out(ind,1) = ones(n,1)*i;
  
  nout = nout + n;
  
  if mod(i,1000) == 0
    disp('Saving check point');
    save out_temp.mat out
  end
  
end

out = out(1:nout,:);


function w2 = adjustWeight(w,hh)
   L = hh(:,2) - hh(:,1);
   lmax = max(L);
   L = L./lmax;

   wmax = max(w);

   scale = (0.5./(1 + exp(-10*L + 4)) + 0.5)./wmax;
   w2 = w.*scale;

function ind = removeConflicts(a,w)
  u1 = zeros(1,640);
  u2 = zeros(1,640);
  W  = -2:2;
  
  [w, iisort] = sort(w);
  ii = zeros(size(a,1),1);
  
  for i = length(w):-1:1
    j = iisort(i);
    c1 = sum(u1(W+a(j,1)));
    c2 = sum(u2(W+a(j,2)));
    c  = c1 + c2;
    
    if c == 0
      %No conflict
      ii(j) = 1;
    end

    u1(a(j,1)) = 1;
    u2(a(j,2)) = 1;
    
  end
  
  ind = find(ii);
