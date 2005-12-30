function tst_posterior()

  map1 = [sqrt(5) 0 .009 0 0 .001;
	  sqrt(4.25) 0 .009 0 0 .001;
	 ];
  map1(1,:) = translate_obs(map1(1,:),[0 0 atan2( 1,2)]);
  map1(2,:) = translate_obs(map1(2,:),[0 0 atan2(+.5,2)]);
 
  map2 = [sqrt(2) 0  .009 0 0 .001;
	  sqrt(1.25) 0  .009 0 0 .001;
	  ];

  map2(1,:) = translate_obs(map2(1,:),[0 0 atan2(+1,1)]);
  map2(2,:) = translate_obs(map2(2,:),[0 0 atan2(+.5,1)]);
  map2(:,3:6) = map2(:,3:6)*.7;


  npx = 50;
  npy = 50;
  dx = 0.5;
  dy = 0.5;
  mx = 1;
  my = 0;
  
  x = linspace(mx - dx, mx + dx, npx);
  y = linspace(my - dy, my + dy, npy);
  w = zeros(npy,npx);
  
  for xi = 1:npx
    for yi = 1:npy
      w(yi,xi) = weight(map1,[x(xi) y(yi) 0],map2);
    end
  end

  cmap = linspace(1,0,64)';
  cmap = [cmap, cmap, cmap];

  if 0
    
  figure
  h =  pcolor(x,y,w);
  set(h,'LineStyle','none');
  hold on
  h = mplot(map1,'rs','r-.');
  set(h,'LineWidth',3);
  map2 = translate_obs(map2,[mx my 0]);
  h = mplot(map2,'go','g-');
  set(h,'LineWidth',3);
    
  h = plot_robot([0 0 0],'rs-');
  set(h,'LineWidth',3);
  h = plot_robot([1 0 0],'go-');
  set(h,'LineWidth',3);
  axis auto
  axis equal
  axis tight
  
  colormap(cmap)
  xlabel('X'); ylabel('Y');
  end
  
  w2 = do_odo([1 0 0],x,y);

  w = w/sum(sum(w));
  w2 = w2/sum(sum(w2));
  
  figure
%  subplot(1,4,1)
  hold on
  h = mplot(map1,'rs','r-.');
  set(h,'LineWidth',3);
  map2 = translate_obs(map2,[mx my 0]);
  h = mplot(map2,'go','g-');
  set(h,'LineWidth',3);
    
  h = plot_robot([0 0 0],'rs-');
  set(h,'LineWidth',3);
  h = plot_robot([1 0 0],'go-');
  set(h,'LineWidth',3);
  
  h = line([mx-dx; mx+dx; mx+dx; mx-dx; mx-dx], ...
	   [my+dy; my+dy; my-dy; my-dy; my+dy]);
  set(h,'LineWidth',3);
  axis auto
  axis equal
  axis tight
  print -depsc /home/kirill/Docs/Shiny/post_example.eps
  
    
  %subplot(1,4,2)
  figure
  h =  pcolor(x,y,w2);
  set(h,'LineStyle','none');
  axis auto
  axis equal
  axis tight
  xlabel('X'); ylabel('Y'); %title('p(x|u)');
  colormap(cmap)
  print -depsc /home/kirill/Docs/Shiny/post_odo.eps

  %subplot(1,4,3)
  figure
  h =  pcolor(x,y,w);
  set(h,'LineStyle','none');
  axis auto
  axis equal
  axis tight
  xlabel('X'); ylabel('Y'); %title('p(x|z)');
  colormap(cmap)
  print -depsc /home/kirill/Docs/Shiny/post_obs.eps

  %subplot(1,4,4)
  figure
  h =  pcolor(x,y,w.*w2);
  set(h,'LineStyle','none');
  axis auto
  axis equal
  axis tight
  xlabel('X'); ylabel('Y'); %title('p(x|u,z)');
  colormap(cmap)
  print -depsc /home/kirill/Docs/Shiny/post_obsodo.eps


  
function w = do_odo(odo,x,y)
  nx = length(x);
  ny = length(y);
  w = zeros(ny,nx);
  
  sa = 25*pi/180/3;
  sd = 0.1/3;
  ma = odo(3);
  md = sqrt(odo(1)^2 + odo(2)^2);
  
  for i = 1:nx
    for j = 1:ny
      [a,d] = cart2pol(x(i),y(j));
      w(j,i) = gauss(d,md,sd)*gauss(a,ma,sa);
    end
  end
  

function v = gauss(x, m, s)
  v = 1./(s*sqrt(2*pi))*exp(-(x-m).*(x-m)/(2*s*s));
  
function w = weight(map1, odo, map2)

  map2 = translate_obs(map2,odo);
  nmap = size(map1,1);
  w = 0;
  
  for i = 1:nmap
    w = w + log_volume(gauss2a(map1(i,:)), gauss2a(map2(i,:)));
  end
  
  w = exp(w);

function lv = log_volume(a1, a2)
  a = a1 + a2;

  lv = 2*log(pi)*(a(6)-a(4)*a(4)/4/a(1) + ...
		  (a(5)-a(3)*a(4)/2/a(1))^2 ...
	         /(a(3)*a(3)/a(1) - 4*a(2))) ...
      /log(-a(1)*(a(3)*a(3)/4/a(1) - a(2)));







