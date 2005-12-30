function [l] = getVerticalLines(I, from, to)
% function [l] = getVerticalLines(I)
%
%  Extract vertical lines from the image (we use left image)
%  I - image rgb or grayscale
%  from - left  most column (in pixels)
%  to   - right most column (in pixels)


K = [0.5 1 0.5];

W_MIN = 100;
MIN_LENGTH = 60;

E = edge(I(:,from:to),'sobel',[],'vertical');

w = sum(E);
w = conv(w,K);
w = w(2:end-1);

ii = find(w > W_MIN);

if isempty(ii)
  l = zeros(0,3);
  return
end

if ii(1) == 1
 ii = ii(2:end);
end

if ii(end) == length(w)
  ii = ii(1:end-1);
end



bb = ispeak(ii,w);

ii = ii(find(bb));

l = zeros(length(ii),3);
l(:,1) = ii';


Kt = K';

%figure;
for i = 1:length(ii)
  x = double(E(:,ii(i) + [-1 0 1]))*Kt;
  x = conv(x,[1 1 1 1 1]);
  x = x(3:end-2);
  
  bb = x > 0.5;
  x(find(bb)) = 1;
  x(find(1-bb)) = 0;
  
  

  [s,f] = getLongestLine(x);
  l(i,2) = s+3;
  l(i,3) = f-3;
  

%  hold off; plot(x); hold on;
%  plot([s,f],[1,1],'ro');
%  drawnow; pause(1);
  
end

L = l(:,3) - l(:,2);
l = l(find(L>MIN_LENGTH),:);
l(:,1) = l(:,1) + from - 1;


function [i1,i2] = getLongestLine(x)
  i = 1;
  maxL = 0;
  i1_max = 0;
  i2_max = 0;
  
  while i < length(x)
    while i < length(x) && ~x(i)
      i = i +1;
    end
    
    istart = i;
    
    while i < length(x) && x(i)
      i = i +1;
    end
    
    L = i - istart;
    
    if L > maxL
      maxL = L;
      i1_max = istart;
      i2_max = i - 1;
    end
    
  end
  
  i1 = i1_max;
  i2 = i2_max;
    

function b = ispeak(ii,w)
 b = w(ii) > w(ii-1) & w(ii) > w(ii+1);
 
 