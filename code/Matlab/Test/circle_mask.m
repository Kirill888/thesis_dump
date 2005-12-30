function mask = circle_mask(sz)
%  function mask = circle_mask(sz)
%
%
%

 mask = zeros(sz);
 
 sz_half = floor(sz/2) + 1;
 r2 = 0.25*sz*sz;
 
 for i = 1:sz
   for j = 1:sz
     dx = i - sz_half;
     dy = j - sz_half;
     
     d2 = dx*dx + dy*dy;
     
     if d2 <= r2
       mask(i,j) = 1;
     end
   end
 end
 

