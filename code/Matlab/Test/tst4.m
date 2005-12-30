function tst4(odo0)
  
  figure;
  
  plot_odo(odo0,[],'ror-b-b-',0.2);
  hold on
  cov0 = [ odo0(4:6); odo0(7:9); odo0(10:12)];
  text(odo0(1),odo0(2),sprintf('%.3e',det(cov0)));
  
  dx = 1;
  dy = 1;
  
  AX = [odo0(1)-dx, odo0(1)+dx, odo0(2)-dy, odo0(2)+dy];
  axis(AX);
  axis equal;
  
  ca = cos(-odo0(3));
  sa = sin(-odo0(3));
  
  xx = collect_points('go',axis,1);
  while ~isempty(xx);
    %convert xx to coordinate frame odo0
    xx(1) = xx(1) - odo0(1);
    xx(2) = xx(2) - odo0(2);
    xx(1) = xx(1)*ca - xx(2)*sa;
    xx(2) = xx(1)*sa + xx(2)*ca;
    
    odo1 = [xx(1),  xx(2), zeros(1,10)];

    
    odo = propogate_odo(odo1,odo0);
    plot_odo(odo,[],'gsg-m-m-',0.2);
    
    cov = [ odo(4:6); odo(7:9); odo(10:12)];
    d = det(cov)
    text(odo(1),odo(2),sprintf('%.3e',d));
    
    xx = collect_points('go',axis,1);
  end
  
  
  
  