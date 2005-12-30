function [ind] = find2(x, y)
% function [ind] = find2(x, y)
%
%  Find elements of x that match any of the elements of y
%

%

ny = length(y);

ind = [];
for i = 1:ny
  ind = [ind; find(x == y(i))];
end



