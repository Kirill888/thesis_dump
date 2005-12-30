function rc = spiral(r0,c0, maxStep)

  step = 1;
  r = r0; c = c0;
  
  rc = [r,c];
  ind = 1;
  dir = 1;
  
  while step <= maxStep
    %Right/Left
    for i = 1:step
      ind = ind + 1;
      c = c + dir;

      rc(ind,1:2) = [r,c];
    end
    
    %Down/Up
    for i = 1:step
      ind = ind + 1;
      r = r - dir;
      rc(ind,1:2) = [r,c];
    end
    
    step = step + 1;
    dir = 0 - dir;
   end
  
