function [h,W] = plot_mapmatch(m1,m2,x,y,a)
% function [h,W] = plot_mapmatch(m1,m2,x,y,a)
%
%

W = zeros(length(y),length(x));

for i = 1:length(x)
  for j = 1:length(y)
    w = map_overlap(m1,m2,[x(i),y(j),a]);
    W(j,i) = exp(sum(w));
  end
end

h = surf(x,y,W);
