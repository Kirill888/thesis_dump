function [] = epipolar_viewer(F,im1, im2, color,  zoomwidth)

% epipolar_viewer allows user selection of an image point in one view and draws
% the epipolar line defined by the selected point and fundamental matrix in a 
% second view.
% Usage:
%   epipolar_viewer(F, im1, im2)
%        where F is the fundamental matrix, im1 and im2 are the 
%        two images such that, if x2 is a point in image 2 and
%        x1 is a point in image 1, x2.'*F*x1 = 0.
%
%       SELECT a point with the left mouse button
%       ZOOM IN with the middle mouse button or SHIFT left button
%                      Keep clicking middle or SHIFT left to move the zoom 
%                     window   around the image.
%       ZOOM OUT with the right mouse button
%       EXIT with ESC or 'x' keys
%       CHANGE focus from image 1 to 2 and back with SPACE or TAB keys
%       CLEAR the display with the 'c' key.
%  Large images  can be reduced in size by dragging the image window
%  without affecting the calculations.
% 
%   epipolar_viewer(F, im1, im2, color)
%       where color is an rgb three vector, with components in the range 0:1,
%      sets the color of the points and lines.  Default is red: color = [1 0 0].
%       Passing an empty matrix [] uses a random color for each point-line pair.
%
%   epipolar_viewer(F, im1, im2, color, zoomsize)
%      where zoomsize is the half width of the zoom box. Default is 100 pixels.

figure(1); clf; imshow(im1); hold on; zoom off; ax1 = axis;
figure(2); clf; imshow(im2); hold on; zoom off; ax2 = axis;

[m1,n1] = size(im1); con1 = conditioning_matrix(n1,m1);
[m2,n2] = size(im2); con2 = conditioning_matrix(n2,m2);
F = inv(con2.')*F*inv(con1);

if nargin == 3
  zoomwidth = 100;
  color = [1 0 0];
end  

if nargin == 4
  zoomwidth = 100;
end  

if isempty(zoomwidth)
    zoomwidth = 100;
end  

colorcycle = 0; 
if isempty(color)
   colorcycle = 1;   
end

figure(1)
button = 1;
while (~(button == 120 | button==27)) 
  fignum = gcf;
  [x,y,button] = ginput(1);
  if (button == 32) | (button == 9)
     if fignum == 1
         figure(2)
     else
         figure(1)
     end
  end 
  if button == 3
    if fignum==1 
        axis(ax1);
    else
        axis(ax2);
     end      
  end
  if button == 2
      axis([x-zoomwidth x+zoomwidth y-zoomwidth y+zoomwidth]);
  end
  if button == 1
    if colorcycle == 1  
       color = rand(1,3); 
    end
    if fignum == 1
        h = plot(x, y,'rx');
        set(h,'MarkerSize', 15);
        set(h,'LineWidth', 2);
        set(h,'Color', color);
        figure(2)    
        l = F*con1*[x; y; 1]; 
        %l = F*[x; y; 1];         
        h = draw_line_bbox(con1.'*l,ax2,'r');
        %h = draw_line_bbox(l,ax2,'r');
        set(h,'LineWidth',2);
        set(h,'Color', color);
        figure(1)
   else
       h = plot(x, y,'rx');
       set(h,'MarkerSize', 15);
       set(h,'LineWidth', 2);   
       set(h,'Color', color);
       figure(1)     
       l = F.'*con2*[x; y; 1]; 
       %l = F.'*[x; y; 1]; 
       h = draw_line_bbox(con2.'*l,ax1,'r'); 
       %h = draw_line_bbox(l,ax1,'r');
       set(h,'LineWidth',2);
       set(h,'Color', color);
       figure(2)        
    end  
 end 
   if button==99
     figure(1); clf; imshow(im1); hold on;
     figure(2); clf; imshow(im2); hold on;
   end
end
hold off

function h = draw_line_bbox(line,bbox,style);
% Draws line such that it is contained
% in the bounding box specified in bbox by
% [xmin xmax ymin ymax] ie axis form.

lxmin = [1; 0;-bbox(1)];
lxmax = [1; 0; -bbox(2)];
lymin = [0; 1; -bbox(3)];
lymax = [0; 1; -bbox(4)];

p1 = dehom(cross(line,lxmin));
p2 = dehom(cross(line,lxmax));
p3 = dehom(cross(line,lymin));
p4 = dehom(cross(line,lymax));

inbox = zeros(1,4);
if p1(2) >= bbox(3) & p1(2) <= bbox(4) 
  inbox(1) = 1; 
end
if p2(2) >= bbox(3) & p2(2) <= bbox(4) 
  inbox(2) = 1; 
end
if p3(1) >= bbox(1) & p3(1) <= bbox(2) 
  inbox(3) = 1; 
end
if p4(1) >= bbox(1) & p4(1) <= bbox(2) 
  inbox(4) = 1; 
end

%inbox
I = find(inbox==1);

if isempty(I)
  disp('Line lies outside bounding box');
  h = [];
else  
  p = [p1 p2 p3 p4];
  h = plot([p(1,I(1)) p(1,I(2))]+1, [p(2,I(1)) p(2,I(2))]+1, style);
end


function x = dehom(y)
  x = y/y(length(y));
  
function [Co] = conditioning_matrix(xsize, ysize)
r = ysize;
c = xsize;

f = (r+c)/2;
dec = [f 0 c/2;
     0 f r/2;
     0 0 1];

Co = inv(dec);

