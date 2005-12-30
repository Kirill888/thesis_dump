function abc = line2abc(l);
% function abc = line2abc(l);
%
%  [x1,y1,x2,y2] ==> a*x + b*y + c = 0
%
%

  nl = size(l,1);
  abc = zeros(nl,3);
  
  dx = l(:,3) - l(:,1);
  dy = l(:,4) - l(:,2);
  
  i1 = find(abs(dx) >= abs(dy));
  a  = - dy(i1)./dx(i1);
  b  = ones(size(i1,1),1);
  
  abc(i1,1:2) = [a,b];

  i2 = find(abs(dx)  < abs(dy));
  b  = - dx(i2)./dy(i2);
  a  = ones(size(i2,1),1);

  abc(i2,1:2) = [a,b];
  
  abc(:,3) = - (abc(:,1).*l(:,1) + abc(:,2).*l(:,2));
  
  
  
  
