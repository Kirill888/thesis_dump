function odo = transit_pose(odo0, trans)
%function odo = transit_pose(odo0, trans)

%
np = size(odo0,1);
nt = size(trans,1);

n = nt*np;
odo = zeros(n,3);

for i = 1:nt
  ind = (1 + (i-1)*np):i*np;
  
  odo(ind,:) = translate_odo(odo0,trans(i,:));
end
