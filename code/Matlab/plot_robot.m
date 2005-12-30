function h = plot_robot(odo,odo_style,odo_r)
%function h = plot_robot(odo,odo_style,odo_r)

    if ishold
       was_hold = 1;
    else
       was_hold = 0;
       
       cla reset
       hold on
    end

    if ~exist('odo_r','var')
      odo_r = 0.1;
    end

    if ~exist('odo_style','var')
      odo_color        = 'r';
    else
      odo_color        = odo_style(1);
    end

    [u,v] = pol2cart(odo(:,3), odo_r*ones(size(odo,1),1));
    
    h = quiver(odo(:,1), odo(:,2), u, v);
    set(h,'Color', odo_color);
    

function old_code
    if ~exist('odo_style','var')
      odo_color        = 'r';
      odo_center_style = 'o';
      odo_line_style   = '-';
    else
      odo_color        = odo_style(1);
      odo_center_style = odo_style(2);
      odo_line_style   = odo_style(3:length(odo_style));
    end

    [lx,ly] = odo2line(odo,odo_r);

    hodo = line(lx,ly);

    set(hodo,'Color',odo_color);
    set(hodo,'LineStyle',odo_line_style);

    if nargout > 0
      h2 = plot(odo(:,1),odo(:,2),[odo_color odo_center_style]);
      h = [hodo;h2];
    else
      plot(odo(:,1),odo(:,2),[odo_color odo_center_style]);
    end

    if ~was_hold
       hold off
    end

function [lx,ly] = odo2line(odo,r)
  n = size(odo,1);

  lx = zeros(2,n);
  ly = zeros(2,n);

  for i = 1: n
    lx(1,i) = odo(i,1);
    ly(1,i) = odo(i,2);

    lx(2,i) = odo(i,1) + r*cos(odo(i,3) ); 
    ly(2,i) = odo(i,2) + r*sin(odo(i,3) ); 

  end
