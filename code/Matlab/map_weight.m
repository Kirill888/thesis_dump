function w = map_weight(poses,links)
%function w = map_weight(poses,links)
%
%

  n = size(links,1);
  w = zeros(n,1);
  
  for i = 1:n
    w(i) = aux1(poses, links(i,:));
  end

  
function w = aux1(poses, link)

  from = poses(link(1),1:3);
  to   = poses(link(2),1:3);
  
  l    = translate_odo(link(3:14),from);
  
  w = md2(to,l);
  
function w = md2(odo,l)
  cov  = [l(4:6); l(7:9); l(10:12)];
  
  xyo  = l(1:3);
  
  dd = zeros(1,3);
  
  dd(1:2)   = xyo(1:2) - odo(1:2);
  
  dd(3) = angle_diff(xyo(3), odo(3));
  
  w = dd*inv(cov)*dd';
  
  

