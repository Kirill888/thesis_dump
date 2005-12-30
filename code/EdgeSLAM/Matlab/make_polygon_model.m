function [x1, x2, tri_pts] = make_polygon_model(im1, im2, x1, x2, tri_pts)
% make_polygon_model(im1, im2)
%
% Allow user to interactively select point correspondances and 
% ploygon faces with the mouse.  Pressing 'p' switches from selecting 
% point correspondances to identifying polygon faces, and <esc> 
% ends the selection process, and exits the function.
%
% Can also be used to append pts and/or polygon faces to an 
% existing set:
%   make_polygon_model(im1, im2, x1, x2) 
%   make_polygon_model(im1, im2, x1, x2, tri_pts)
%
figure(1); clf; imshow(im1); hold on; zoom off; ax1 = axis;
figure(2); clf; imshow(im2); hold on; zoom off; ax2 = axis;
figure(1);
if nargin == 4
    tri_pts = []; 
    format_figure(x1(:,1), x1(:,2), [0 1 1], [1:size(x1,1)]');  
    figure(2); 
    format_figure(x2(:,1), x2(:,2), [0 1 1], [1:size(x1,1)]');
elseif nargin == 5
    for ii = 1:size(tri_pts,1)
        fill(x1(tri_pts(ii,1:3),1), x1(tri_pts(ii,1:3),2), [1 1 0]);
    end; %for
    format_figure(x1(:,1), x1(:,2), [0 1 1], [1:size(x1,1)]');  
    figure(2); 
    for ii = 1:size(tri_pts,1)
        fill(x2(tri_pts(ii,1:3),1), x2(tri_pts(ii,1:3),2), [1 1 0]);
    end; %for
    format_figure(x2(:,1), x2(:,2), [0 1 1], [1:size(x1,1)]');
elseif nargin == 2
    x1 = [];
    x2 = [];
    tri_pts = [];
end; %else
figure(1);

[m1,n1] = size(im1); 
[m2,n2] = size(im2); 
if nargin == 2
    zoomwidth = 100;
end  
if isempty(zoomwidth)
    zoomwidth = 100;
end  
color_ind = 1;   
color_list = {[0 1 1], [0 1 0], [1 0 0], [1 1 0], [1 0 1], [0 0.5 1], [1 0.5 0.5], [0.5 1 0], [1 0.5 0]};
color = color_list{color_ind};
figure(1)
button = 1;
new_patch = 1;
patch_start_pt = 0;
mode = 'select_points';
new_poly = [];
tri_pts = [];
while (~(button == 120 | button==27))                   % - Exit if ESC or 'x' pressed
    [x,y,button] = ginput(1);                           % - get input from mouse or keyboard
    zoom_check(button, ax1, ax2, zoomwidth, x, y)  
    mode = p_check2(button,mode);
    switch_fig_check(button,mode);
    figure(1); title('Choose a point in this view');
    if button == 1
        disp('button 1 pressed');
        switch lower(mode)
        case 'select_points'
            [x1,x2] = select_point(x, y, ax1, ax2, zoomwidth, [0 1 1], x1, x2);
            %[color, color_ind] = change_color(color_list, color_ind);
            title('Choose a point, or <p> to switch to polygon selection mode, or <esc> to exit');        
        case 'select_polygons'
            disp('mode:select_ploygons')
            new_poly = select_polygon(x, y, x1, x2, new_poly);
            if length(new_poly)==3
                tri_pts = [tri_pts; new_poly];
                fill(x1(new_poly,1), x1(new_poly,2), [1 1 0]);
                format_figure(x1(new_poly,1), x1(new_poly,2), [0 1 1], new_poly');    
                new_poly = [];
            end; %if
        end; %switch
    end; %if
    if button==99
        figure(1); clf; imshow(im1); hold on;
        figure(2); clf; imshow(im2); hold on;
    end
end
hold off


%------------------------------------------------------------------------------------------
function zoom_check(button, ax1, ax2, zoomwidth, x, y)
if button == 3                                        % - if 3rd mouse button pressed, zoom to original size
    if gcf==1 
        axis(ax1);
    else
        axis(ax2);
    end      
end
if button == 2                                        % - if 2nd mouse button pressed, zoom in
    axis([x-zoomwidth x+zoomwidth y-zoomwidth y+zoomwidth]);
end


%------------------------------------------------------------------------------------------
function switch_fig_check(button, mode)
if button == 32                                        % - if spacebar pressed, 
    if mode == 'select_polygons'
        if gcf==1 
            figure(2);
        else
            figrue(1);
        end  
    else
        disp('Can only switch figures in polygon selection mode.');
    end;
end

%------------------------------------------------------------------------------------------
function mode = p_check2(button, mode);
if button == 112                                    % - if 'p' pressed switch modes
    switch lower(mode)
    case 'select_points'
        disp('switching to polygon selection mode');  
        mode = 'select_polygons';
        xlabel('Select polygon');
    case 'select_polygons'
        disp('switching to point selection mode');
        mode = 'select_points';
        xlabel('Select point correspondance');
    end  
end; %switch


%------------------------------------------------------------------------------------------
function [x1,x2] = select_point(x,y,ax1,ax2,zoomwidth,color,x1,x2);
disp('mode:select_points')
x1(end+1,1:2) = [x y];
format_figure(x,y,color,size(x1,1));       
title('Now identify the same point in the other view');
figure(2); title('Choose a point in this view');
button = 0;
while button ~= 1
    [x,y,button] = ginput(1);
    zoom_check(button, ax1, ax2, zoomwidth, x, y)  
end; %while
x2(end+1,1:2) = [x y];
format_figure(x,y,color,size(x2,1));    
title('');
figure(1);


%------------------------------------------------------------------------------------------
function format_figure(x,y,color,c);
h = plot(x, y,'ro');
set(h,'MarkerSize', 5);
set(h,'LineWidth', 2);   
set(h,'Color', color);
H = text(x+4, y+4,int2str(c));
set(H,'Color',color)
set(H,'FontSize',10)


%------------------------------------------------------------------------------------------
function [color, color_ind] = change_color(color_list, color_ind);
color_ind = mod(color_ind, length(color_list))+1;
color = color_list{color_ind};


%------------------------------------------------------------------------------------------
function new_poly = select_polygon(x, y, x1, x2, new_poly); 
disp('mode:select_polygons')
dist = sqrt((x1(:,1)-x(1)).^2 + (x1(:,2)-y(1)).^2);
[m closest] = min(dist);
xx1 = x1(closest,1);    yy1 = x1(closest,2); 
xx2 = x2(closest,1);    yy2 = x2(closest,2); 
figure(1),
h = plot(xx1,yy1,'ro'); set(h,'MarkerSize', 5); set(h,'LineWidth', 2);  
title('Selected point highlighted in red');
figure(2),
h = plot(xx2,yy2,'ro'); set(h,'MarkerSize', 5); set(h,'LineWidth', 2);  
title('Selected point highlighted in red');
figure(1);
new_poly = [new_poly, closest];

%------------------------------------------------------------------------------------------
function show_planar_image(im1_rect, x_rect_ic);
% display unwarped image and tie points
%
figure, imagesc(im1_rect), hold on,
%plot(x_rect(1,:),x_rect(2,:),'rx'); text(x_rect(1,:),x_rect(2,:),int2str([1:3]'));
h = line([x_rect_ic(1,:), x_rect_ic(1,1)], [x_rect_ic(2,:), x_rect_ic(2,1)]);
%h = plot(x_rect_ic(1,:),x_rect_ic(2,:),'gx'); 
set(h,'MarkerSize', 5); 
set(h,'LineWidth', 2);  
set(h,'Color',[0 1 0])
h = plot(x_rect_ic(1,:),x_rect_ic(2,:),'ko');
set(h,'MarkerSize', 5); 
set(h,'LineWidth', 10);  
h = text(x_rect_ic(1,:),x_rect_ic(2,:),int2str([1:3]'));
set(h,'Color',[1 1 1])
set(h,'FontSize',15)
set(h,'FontWeight','light')
set(h,'HorizontalAlignment','center');
axis image; %axis off; 
colormap(gray);
