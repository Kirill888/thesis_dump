function obs = translate_obs(obs, odo0)
% function obs = translate_obs(obs, odo0)
%

%
  a = odo0(3);
  R = [ cos(a), sin(a);
       -sin(a), cos(a)];
  
  obs(:,1:2) = obs(:,1:2)*R;
  obs(:,1)   = obs(:,1) + odo0(1);
  obs(:,2)   = obs(:,2) + odo0(2);

  if size(obs,2) >= 6
    nobs = size(obs,1);
    for i = 1:nobs
      p = [obs(i,3:4); obs(i,5:6)];
      p = R'*p*R;
      obs(i,3:4) = p(1,:);
      obs(i,5:6) = p(2,:);
    end
  end
