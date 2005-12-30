function batch_play2(data,frame)
% function batch_play2(data,frame)
%  
%
%

%

global X_MAPPING Y_MAPPING C1 C2

global LEFT_MASK
global RIGHT_MASK
global H_OFFSET

fig = figure;
subplot(2,1,1);
subplot(2,1,2);

fr = data(1,1);
to = data(end,1);

if nargin > 1
  fr = frame;
end


i = fr;

while i <= to
  disp(sprintf('Loading Images: %d',i));
  
  I1 = imread(sprintf(LEFT_MASK,i));
  I2 = imread(sprintf(RIGHT_MASK,i));

  II = [I1, I2];
  figure(fig);
  subplot(2,1,1); hold off;
  imshow(II); hold on;

  ind = find(data(:,1) == i);
  l1 = data(ind,2);
  l2 = data(ind,3);
  h  = data(ind,5:6);
  
  line([l1,l1]',h','LineWidth',2);

  line(640 + [l2,l2]', h' + H_OFFSET,'LineWidth',2);
  
  subplot(2,1,2);;
  
  lx = zeros(length(l1),3);
  ly = zeros(length(l1),3);
  
  lx(:,1) = C1(1)/1000; ly(:,1) = C1(2)/1000;
  lx(:,3) = C2(1)/1000; ly(:,3) = C2(2)/1000;

  xy = lines2points(l1,l2);
  
  cla;
  lx(:,2) = xy(:,1);
  ly(:,2) = xy(:,2);
  
  line(lx',ly','LineWidth',2);hold on;
  plot(lx(:,2), ly(:,2),'ro');
  mplot(xy,'b.','b-');

  axis equal
  axis([-.100 20 -3 3]);
  drawnow

  k = menu('ACTION','>','<','>>>>>','<<<<<<','###','QUIT');

  switch(k)
   case 1
    i = i + 1;
   case 2
    i = i - 1;
   case 3
    i = i + 10;
   case 4
    i = i - 10;
   case 5
    i = input('Enter frame:');
   case 6
    return
   otherwise
    return
  end
  
end
