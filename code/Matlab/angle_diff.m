function c = angle_diff(a,b)
%function c = angle_diff(a,b)
   c = a - b;

   [n,m] = size(c);
   for i = 1:n*m
     while c(i) < -pi
       c(i) = c(i) + 2*pi;
     end

     while c(i) > pi
       c(i) = c(i) - 2*pi;
     end
   end
