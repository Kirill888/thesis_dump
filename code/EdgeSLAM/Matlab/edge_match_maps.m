function L = edge_match_maps(gmap,obs, m1,m2,m2in1)
% function edge_match_maps(gmap,obs, m1,m2,m2in1)
%
%
%
%

minCorr = 16348*0.7;


map1 = gmap.maps(m1);
map2 = gmap.maps(m2);

map1.map(:,3:6) = map1.map(:,3:6)*9;
map2.map(:,3:6) = map2.map(:,3:6)*9;

t1 = getTemplates(map1,obs);
t2 = getTemplates(map2,obs);

if 0
figure(1)
subplot(2,1,1);
plot(t1','.-');
axis([1,21,-100,100]);

subplot(2,1,2);
plot(t2','.-');
axis([1,21,-100,100]);
end

nm1 = size(t1,1);
nm2 = size(t2,1);

C = zeros(nm1,nm2);

for i1 = 1:nm1
  for i2 = 1:nm2
    C(i1,i2) = t1(i1,:)*t2(i2,:)';
  end
end

ii = find(C < 0);
C(ii) = 0;


figure(1)
subplot(2,2,1); hold off
imagesc(C);
ylabel(sprintf('Map %d',m1));
xlabel(sprintf('Map %d',m2));
title('Landmark correlations');

subplot(2,2,2); hold off
ii = find(C > minCorr);
C0 = zeros(size(C));
C0(ii) = 1;
imagesc(C0);
ylabel(sprintf('Map %d',m1));
xlabel(sprintf('Map %d',m2));
title(sprintf('Threshholded at %d(%.3f)',floor(minCorr),minCorr/128/128));

colormap('gray');

subplot(2,4,5); hold off
plot(t1','.-');
axis([1,21,-100,100]);
title(sprintf('Avg tmplates %d',m1));

subplot(2,4,6); hold off
plot(t2','.-');
title(sprintf('Avg tmplates %d',m2));
axis([1,21,-100,100]);

subplot(4,2,6)
bar(1:nm1,sum(C0'));
axis([1,max([nm1,nm2]),0,max([nm1,nm2])]);
ylabel('Num. matches')
xlabel('Landmark index');
title(sprintf('map %d',m1));

subplot(4,2,8)
bar(1:nm2,sum(C0));
axis([1,max([nm1,nm2]),0,max([nm1,nm2])]);
ylabel('Num. matches')
xlabel('Landmark index');
title(sprintf('map %d',m2));

disp('Computing edges');
e1 = compute_edges(map1.map);
e2 = compute_edges(map2.map);

e1 = sortrows(e1,1);
e2 = sortrows(e2,1);

ii = find(e1(:,1) < 0.2);
e1(ii,1:2) = nan;

ii = find(e2(:,1) < 0.2);
e2(ii,1:2) = nan;

%e1(:,2) = e1(:,2) + 0.1;
%e2(:,2) = e2(:,2) + 0.1;

ne1 = size(e1,1);
ne2 = size(e2,1);

MD2 = zeros(ne1,ne2);
E   = MD2;
P1  = MD2;
A   = MD2;
X   = MD2;
Y   = MD2;

for i = 1:ne1
  dd = e1(i,1) - e2(:,1);
  ss = e1(i,2) + e2(:,2);
  
  MD2(i,:) = [dd.*dd./ss]';

  %  E(i,:)   = 1./sqrt(ss').*exp(-0.5*MD2(i,:));
  E(i,:)   = -0.5*(log(ss') + MD2(i,:));
  P1(i,:) = C0(e1(i,3),e2(:,3)).*C0(e1(i,4),e2(:,4));

  A(i,:) = [e1(i,7) - e2(:,7)]';
  ca = cos(A(i,:));
  sa = sin(A(i,:));
  x = e2(:,5)'.*ca - e2(:,6)'.*sa;
  y = e2(:,5)'.*sa + e2(:,6)'.*ca;

  X(i,:) = e1(i,5) - x;
  Y(i,:) = e1(i,6) - y;
end

emax = max(E(:))
E = E-emax;
E = exp(E);
max(E(:))

E(find(isnan(E))) = 0;
PA = exp(-0.5*pose_md2(X,Y,A,m2in1));

P = E.*P1.*PA;
P = P./max(P(:));

Edge_w = P(:);
Edge_csw = cumsum(Edge_w);

L = zeros(nm1,nm2);

for i = 1:ne1
  l1_1 = e1(i,3);
  l1_2 = e1(i,4);

  for j = 1:ne2
    l2_1 = e2(j,3);
    l2_2 = e2(j,4);
    
    v = P(i,j);
    
    L(l1_1,l2_1) = L(l1_1,l2_1) + v;
    L(l1_2,l2_2) = L(l1_2,l2_2) + v;
  end

end

L = L.*C;
L = L./max(L(:));

ii = find(L(:) < 0.3);
L2 = L;
L2(ii) = 0;

figure(2)

subplot(2,2,1); hold off
imagesc(-E);
subplot(2,2,2); hold off
imagesc(1-P);
subplot(2,2,3); hold off
imagesc(L);

subplot(2,2,4); hold off
ii = cumsum_sample(Edge_csw,1000);
if 1
odo = [X(ii),Y(ii),A(ii)];
plot_robot(odo,'r');hold on
plot_odo(m2in1,[],'b.b-b-b-b-b-',1);
axis equal
else
  plot(ii,PA(ii),'r.',ii,P(ii),'b.');
end


sum(L2(:)>0)
colormap('gray');



    
function t = getTemplates(m,obs)
  t = zeros(size(m.map,1),21);

  for i = 1:size(m.map,1)
    t(i,:) = mean(obs(m.map2obs{i},10:end));
  end


function md2=angle_md2(a, a0, saa)

  da = angle_diff(a,a0);
  md2 = da.*da./saa;
  
function md2=pose_md2(x,y,a, xyr)

  dx = x - xyr(1);
  dy = y - xyr(2);
  da = angle_diff(a,xyr(3));

  COV = xyr([4:6;7:9;10:12]);

  ICOV = inv(COV);
  
  md2 = dx.*dx.*ICOV(1) + dy.*dy.*ICOV(5) + da.*da.*ICOV(9)+ ...
	2*dx.*dy.*ICOV(2) + 2.*dx.*da.*ICOV(3) + 2.*dy.*da.*ICOV(6);

  

 
function smpl = cumsum_sample(w_cumSum, n)
  sum = w_cumSum(end);
  pos = rand(1)*sum;
  step = sum/n;
  ind = 1;
  
  smpl = zeros(n,1);
  
  for i = 1:n

    pos = pos + step;
    
    if pos > sum
      pos = pos - sum;
      ind = 1;
    end
    
    while pos > w_cumSum(ind)
      ind = ind + 1;
    end
    
    smpl(i) = ind;
      
  end
  

function e = compute_edges(m)
% function e = compute_edges(m)
%
%
%  e = [r,Srr,l1,l2,x,y,a]

  n = size(m,1);
  ne = n*(n-1);
  
  if ne <= 0
    e = zeros(0,7);
    return
  end
  
  from = zeros(ne,1);
  to   = zeros(ne,1);
  
  W = 1:(n-1);
  for i = 1:n
    from(W+(i-1)*(n-1)) = ones(n-1,1)*i;
      to(W+(i-1)*(n-1)) = setdiff(1:n,i);
  end
  
  
  %Compute length
  
  dx = m(to,1) - m(from,1);
  dy = m(to,2) - m(from,2);
  dx2 = dx.*dx;  dy2 = dy.*dy; dxdy = dx.*dy;
  
  r2 = dx2 + dy2;
  r  = sqrt(r2);
  
  sxx = m(from,3) + m(to,3);
  sxy = m(from,4) + m(to,4);
  syy = m(from,6) + m(to,6);
  
  Srr = dx2.*sxx + dy2.*syy + 2*dxdy.*sxy./r2;
  
  ii = find(Srr < 0);
  Srr(ii) = nan;
  
  x = 0.5*(m(to,1) + m(from,1));
  y = 0.5*(m(to,2) + m(from,2));
  a = atan2(dy,dx);

  e = [r,Srr,from,to,x,y,a];
