function odo = translate_odo(odo, odo0)
  a = odo0(3);
  R = [ cos(a), sin(a);
       -sin(a), cos(a)];
  
  odo(:,1:2) = odo(:,1:2)*R;
  odo(:,1)   = odo(:,1) + odo0(1);
  odo(:,2)   = odo(:,2) + odo0(2);
  odo(:,3)   = odo(:,3) + a;
  
  if size(odo,2) == 12
    T  = [R, [0;0]; [ 0 0 1]];
    T_ = T';
    ind = 3 + [1 4 7 2 5 8 3 6 9];

    for i = 1:size(odo,1)
      cov = [ odo(i,4:6); odo(i,7:9); odo(i,10:12)];
      cov = T*cov*T_;

      odo(i,ind) = cov(:)';
    end
  end
  

