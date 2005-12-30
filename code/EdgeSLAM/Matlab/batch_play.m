function batch_play(data,fr,to,t_pause)
% function batch_play(data,fr,to)

global X_MAPPING Y_MAPPING C1 C2
global H_OFFSET

global LEFT_MASK 
global RIGHT_MASK 

fig = figure;
subplot(2,1,1);
subplot(2,1,2);

if nargin < 2
  fr = data(1,1);
end

if nargin < 3
  to = data(end,1);
end

if nargin < 4
  t_pause = -1;
end


for i = fr:to
  disp(sprintf('Loading Images: %d',i));
  
  I1 = imread(sprintf(LEFT_MASK,i));
  I2 = imread(sprintf(RIGHT_MASK,i));

  II = [I1, I2];
  figure(fig);
  subplot(2,1,1); hold off;
  imshow(II); hold on;

  ind = find(data(:,1) == i);
  l1 = data(ind,2:4);
  l2 = data(ind,5:7);
  data(ind,:)
  
  ind = find(l2(:,1) > 0);
  
  line([l1(ind,1),l1(ind,1)]',[l1(ind,2),l1(ind,3)]','LineWidth',2);

  line([l2(ind,1),l2(ind,1)]' + 640, ...
       [l1(ind,2),l1(ind,3)]'+H_OFFSET,'LineWidth',2);
  
  ind = find(l2(:,1) == 0);
  line([l1(ind,1),l1(ind,1)]',[l1(ind,2),l1(ind,3)]',...
       'LineWidth',1,'Color','r','LineStyle',':');
  
  subplot(2,1,2);;
  ind = find(l2(:,1) > 0);
  
  lx = zeros(length(ind),3);
  ly = zeros(length(ind),3);
  
  lx(:,1) = C1(1)/1000; ly(:,1) = C1(2)/1000;
  lx(:,3) = C2(1)/1000; ly(:,3) = C2(2)/1000;

  xy = lines2points(l1(ind,1),l2(ind,1));
  
%  for i = 1:length(ind);
%    a = l1(ind(i),1);
%    b = l2(ind(i),1);
%    
%    lx(i,2) = X_MAPPING(a,b);
%    ly(i,2) = Y_MAPPING(a,b);
%  end

  cla;
  lx(:,2) = xy(:,1);
  ly(:,2) = xy(:,2);
  
  line(lx',ly','LineWidth',2);hold on;
  plot(lx(:,2), ly(:,2),'ro');
  mplot(xy,'b.','b-');


  axis equal
  axis([-0.1 10 -2 2]);

  if t_pause < 0
    pause;
  else
    pause( t_pause );
  end
  
end
