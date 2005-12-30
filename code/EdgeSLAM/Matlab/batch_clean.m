function out = batch_clean(data)
% function out = batch_clean(data)

fr = data(1,1);
to = data(end,1);

out = zeros(size(data,1),6);
nout = 0;

for i = fr:to
  ind = find(data(:,1) == i);
  hh = data(ind,3:4);
  l1 = data(ind,2);
  l2 = data(ind,5:7);
  w  = data(ind,8:10);
  
  pairs = [l1, l2(:,1), w(:,1), hh;...
	   l1, l2(:,2), w(:,2), hh;...
	   l1, l2(:,3), w(:,3), hh];

  ind = find(pairs(:,2) > 0);
  pairs = pairs(ind,:);

  w_ = adjustWeight(pairs(:,3),pairs(:,4:5));
  
  

  ii = removeConflicts(pairs,w_);
  pairs = pairs(ii,:);
  
  n = size(pairs,1);
  
  ii = (nout+1):(nout+n);
  
  out(ii,:) = [i*ones(n,1), pairs];
  
  nout = nout + n;
end

out = out(1:nout,:);

function w2 = adjustWeight(w,hh)
   L = hh(:,2) - hh(:,1);
   lmax = max(L);
   L = L./lmax;

   wmax = max(w);

   scale = (0.5./(1 + exp(-10*L + 4)) + 0.5)./wmax;
   w2 = w.*scale;

function ind = removeConflicts(a,w)
  u1 = zeros(1,640);
  u2 = zeros(1,640);
  W  = -2:2;
  
  [w, iisort] = sort(w);
  ii = zeros(size(a,1),1);
  
  for i = length(w):-1:1
    j = iisort(i);
    c1 = sum(u1(W+a(j,1)));
    c2 = sum(u2(W+a(j,2)));
    c  = c1 + c2;
    
    if c == 0
      %No conflict
      ii(j) = 1;
    end

    u1(a(j,1)) = 1;
    u2(a(j,2)) = 1;
    
  end
  
  ind = find(ii);
