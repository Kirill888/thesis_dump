function ii = findPeaks(x, minx)
% function ii = findPeaks(x, minx)

X = [minx-1; x; minx-1];

ii = find(X > minx);
bb = X(ii) > X(ii+1) & X(ii) > X(ii-1);
ii = ii(find(bb))-1;
