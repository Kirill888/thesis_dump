function pix_offset = tst_template(M,n)
% function pix_offset = tst_template(M,n)
%
%


m = floor(M/2);

pix_offset = zeros(M*M,1);
i = 1;

for c = -m:1:m
  for r = -m:1:m
    pix_offset(i) = c*n + r;
    i = i + 1;
  end
end



