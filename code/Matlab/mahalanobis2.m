function m2 = mahalanobis2(obs,obsp,map)
%function m2 = mahalanobis2(obs,obsp,map)

  n = size(obs,1);
  m = size(map,1);

  m2 = zeros(n,m);

  for i = 1:n
     p_inv = inv(obsp(:,:,i));
     for j = 1:m
	x = obs(i,:) - map(j,:);
        m2(i,j) = x*p_inv*x';
     end
  end
