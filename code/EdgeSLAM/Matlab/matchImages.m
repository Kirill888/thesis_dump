function [l1,l2,l2_w] = matchImages(do_plot)
%function matchImages(I1,I2,do_plot)

%1. Extract Lines from Image 1
%2. Find candidate matches in image 2 (upto 3 matches are returned)

global I1
global I2
global H_OFFSET
global N_ROWS
global N_COLS

if nargin < 1
  do_plot = 1;
end



if size(I1,3) == 3
  l1 = getVerticalLines(I1(:,:,1), 10, N_COLS-10);
else
  l1 = getVerticalLines(I1, 10, N_COLS-10);
end

ii = removeClutter(l1(:,1),10);
l1 = l1(ii,:);

%Make sure l2 falls within the visible area.
if H_OFFSET < 0
  y2 = l1(:,2) + H_OFFSET;
  ii = find(y2 < 1);
  l1(ii,2) = -H_OFFSET+1;
else
  y2 = l1(:,3) + H_OFFSET;
  ii = find(y2 > N_ROWS);
  l1(ii,3) = N_ROWS - H_OFFSET;
end

nlines = size(l1,1);
l2 = zeros(nlines,3); %Up to three candiates
l2_w = zeros(nlines,3);

for i = 1:nlines
  tmp = findLines(l1(i,:));
  nmatches = size(tmp,1);
  
  if nmatches >= 4 || nmatches == 0 %Discard
    %disp(sprintf('Discarding Line: %d (%d)',i,nmatches));
  else
   tmp = sortrows(tmp,2);
   tmp = tmp(end:-1:1,:); 
   ntake = min(3,nmatches);
   l2(i,1:ntake) = tmp(1:ntake,1)';
   l2_w(i,1:ntake) = tmp(1:ntake,2)';
  end
end



if do_plot
  II = [I1, I2];
  figure;
  imshow(II); hold on;
  ind = find(l2(:,1) > 0);
  
  line([l1(ind,1),l1(ind,1)]',[l1(ind,2),l1(ind,3)]','LineWidth',2);

  line([l2(ind,1),l2(ind,1)]' + 640, ...
       [l1(ind,2),l1(ind,3)]','LineWidth',2);
  
  ind = find(l2(:,1) == 0);
  line([l1(ind,1),l1(ind,1)]',[l1(ind,2),l1(ind,3)]',...
       'LineWidth',1,'Color','r','LineStyle',':');
  
  figure;
  ind = find(l2(:,1) > 0);
  global X_MAPPING Y_MAPPING C1 C2
  
  lx = zeros(length(ind),3);
  ly = zeros(length(ind),3);
  
  lx(:,1) = C1(1); ly(:,1) = C1(2);
  lx(:,3) = C2(1); ly(:,3) = C2(2);
  
  for i = 1:length(ind);
    a = l1(ind(i),1);
    b = l2(ind(i),1);
    
    lx(i,2) = X_MAPPING(a,b);
    ly(i,2) = Y_MAPPING(a,b);
  end

  line(lx',ly','LineWidth',2);hold on;
  plot(lx(:,2), ly(:,2),'ro');
  
  axis equal
  
end

return


function ii = removeClutter(x, THRESHOLD)
i = 1;
n = length(x);
good = ones(n,1);

while i < n-1
  if x(i+1) - x(i) < THRESHOLD
    good(i+1) = 0;
    good(i) = 0;
  end
  i = i +1;
end

ii = find(good);