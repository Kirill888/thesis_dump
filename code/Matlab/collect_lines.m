function [lines,h] = collect_lines()
% function [lines,h] = collect_lines()
%
%

 nl = 0;
 
 np = 0;
 hold on

 [x,y,b] = ginput(1);
 hp = [];
 hl = [];
 
 point_marker = 'bo';
 
 while (length(b) == 1) & (b ~= 2) 
   np = np + 1;
   xy(np,:) = [x,y];

   if isempty(hp)
     hp = plot(x,y,point_marker);
   else
     set(hp,'XData',xy(:,1),'YData',xy(:,2));
   end
   
   if isempty(hl) && np > 1
     hl = line(xy(:,1),xy(:,2));
   else
     set(hl,'XData',xy(:,1),'YData',xy(:,2));
   end
   
     
   
   [x,y,b] = ginput(1);
 end

 h = [hl,hp]; 
 nl = np - 1;
 
 if nl <= 0
   lines = zeros(0,4);
 else
   lines = zeros(nl,4);
 end
 
 for i = 1:nl
   lines(i,1:2) = xy(i,:);
   lines(i,3:4) = xy(i+1,:);
 end
 
