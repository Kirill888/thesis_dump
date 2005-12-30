function z = propagate_obs(obs, odo)
% function z = propagate_obs(obs, odo)
%
%

%
  
  
  nobs = size(obs,1);
  
  z_ = translate_obs(obs,odo);
 
  ca = cos(odo(3));
  sa = sin(odo(3));
%  det(Sx)
  z  = z_;
  
  ind = 2 + [ 1 3 2 4];
  for i = 1:nobs
    
    a = -obs(i,1)*sa - obs(i,2)*ca;
    b =  obs(i,1)*ca - obs(i,2)*sa;
    T = [1 0 a; 
	 0 1 b];
    
    Sz_ = [z_(i, 3:4); z_(i, 5:6)];
    
    %    zxx = T*Sx*T'
    %    sqrt(det(zxx))
    
    if length(odo) > 3
      Sx = [ odo(4:6); odo(7:9); odo(10:12)];
      Sz  = T*Sx*T' + Sz_;
    else
      Sz = Sz_;
    end

    z(i, ind) = Sz(:)';
  end
  
    