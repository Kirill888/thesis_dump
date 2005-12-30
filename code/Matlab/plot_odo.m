function h = plot_odo(robot,cov_,style,r)
%  function plot_odo(odo,cov,style,r)
%

%
  
  if isempty(cov_)
    if size(robot,2) ~= 12
      error('Wrong odo size');
    end
  end

  np = size(robot,1);

  h = [];

  for i = 1:np
    
    if isempty(cov_)
      cov = [robot(i,4:6); 
	     robot(i,7:9);
	     robot(i,10:12)];
      odo = robot(i,1:3);
    else
      odo = robot(i,1:3);
      cov = cov_(:,:,i);
    end
    
    style_dot     = style(1:2);
    style_line    = style(3:4);
    style_ellipse = style(5:6);
    style_angle   = style(7:8);
    
    xr(1) = odo(1);
    yr(1) = odo(2);
    xr(2) = odo(1) + r*cos(odo(3));
    yr(2) = odo(2) + r*sin(odo(3));

    cov_xy = cov(1:2,1:2);
    [xe ye] = ellipse(odo,cov_xy);
    
    do = 3*sqrt(cov(3,3));
    o1 = angle_diff(odo(3), do);
    o2 = angle_diff(odo(3),-do);
    
    if 0
      disp(sprintf('o = %f do = %f => %f %f' ...
		   ,odo(3)*180/pi ...
		   ,do*180/pi ...
		   ,o1*180/pi ...
		   ,o2*180/pi ));
    end
    
    xa(1) = odo(1) + r*cos(o1);
    ya(1) = odo(2) + r*sin(o1);
    xa(2) = odo(1);
    ya(2) = odo(2);
    xa(3) = odo(1) + r*cos(o2);
    ya(3) = odo(2) + r*sin(o2);
    
    hold on
    if nargout > 0
      h_ = plot( odo(1),odo(2),style_dot ...
		,xr,yr,style_line       ...
		,xe,ye,style_ellipse    ...
		,xa,ya,style_angle  	      );

      h = [h;h_];
    else
      plot( odo(1),odo(2),style_dot ...
	    ,xr,yr,style_line       ...
	    ,xe,ye,style_ellipse    ...
	    ,xa,ya,style_angle  	      );
      
    end
    
  end
  
  

