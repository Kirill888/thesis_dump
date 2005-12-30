function w = map_overlap(m1,m2, odo)
% function w = map_overlap(m1,m2, [odo])
%
%
%

nm = size(m1,1);

if nargin > 2
  nodo = size(odo,1);
  w = zeros(nodo,nm);
  for i = 1:nodo
    odo0 = odo(i,:);
    m2_ = translate_obs(m2,odo0);
    
    w(i,:) = map_overlap(m1,m2_);
  end

  return
end

for i = 1:nm
  p1   =  m1(i,1:2)';
  cov1 = [m1(i,3:4); m1(i,5:6)]; 
  p2   =  m2(i,1:2)';
  cov2 = [m2(i,3:4); m2(i,5:6)]; 
  
  %w(i)      = vol2(p1,cov1,p2,cov2);
  w(i)  = vol2_log(p1,cov1,p2,cov2);
end



function v = vol2_log(m1,Cov1, m2, Cov2)
% function v = vol2_log(m1,Cov1, m2, Cov2)
%
%

n = length(m1);

A = Cov1 + Cov2;
b = m1 - m2;

v = -0.5*(n*log(2*pi) + log(det(A)) + b'*inv(A)*b);

if 0
K1 = inv(Cov1);
K2 = inv(Cov2);

C = inv(K1 + K2);
B = K1*C*K2;

n = length(m1);

v = log(1/((2*pi)^(n/2))) + 0.5*log(det(C)/(det(Cov1)*det(Cov2)))...
    -0.5*(m1-m2)'*B*(m1-m2);

end

function v = vol2(m1,Cov1, m2, Cov2)
% function v = vol2(m1,Cov1, m2, Cov2)
%
%


K1 = inv(Cov1);
K2 = inv(Cov2);

C = inv(K1 + K2);
B = K1*C*K2;

n = length(m1);

v = 1/((2*pi)^(n/2))*...
    sqrt(det(C)/(det(Cov1)*det(Cov2)))...
    *exp(-0.5*(m1-m2)'*B*(m1-m2));

