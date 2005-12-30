function odo = propagate_odo(odo,odo0)
% function odo = propagate_odo(odo,odo0)
  
  odo_ = translate_odo(odo,odo0);
  
  T = eye(3);
  cov0 = [odo0(4:6); odo0(7:9); odo0(10:12)];
  ca = cos(odo0(3));
  sa = sin(odo0(3));
  
  ind = 3 + [1 4 7 2 5 8 3 6 9];
       
  for i = 1:size(odo,1)
    x = odo(i,1);
    y = odo(i,2);
    
    T(1,3) = -x*sa - y*ca;
    T(2,3) =  x*ca - y*sa;
    
%    T
    cov0_ = T*cov0*T';
%    cov0_ - cov0
    
    odo_(i,ind) = odo_(i,ind) + cov0_(:)';
  end
  
  
  odo = odo_;
  