function odo0 = zero_odo(odo)
%function odo0 = zeroOdo(odo)
  
  x0 = odo(1,1);
  y0 = odo(1,2);
  a0 = odo(1,3);
  
  odo0 = [ odo(:,1)-x0, odo(:,2)-y0, odo(:,3)];
  
  odo0 = translate_odo(odo0,[0 0 -a0]);
  