function x = mycorr2(T,I);
% function x = mycorr2(T,I);

w = size(T,2);
w_2 = floor(w/2);

x = zeros(size(I,1),1);
T = normalise(T);
Ts = sqrt(T(:)'*T(:));

for i = w_2+1:(size(I,2)-w_2)
  i0 = i - w_2;
  T2 = normalise(I(:,i0:(i0+w-1),:));
  x(i) = T(:)'*T2(:)./Ts./sqrt(T2(:)'*T2(:));
end


function In = normalise(I)
  m = mean(I(:));
  In = double(I) - m;
