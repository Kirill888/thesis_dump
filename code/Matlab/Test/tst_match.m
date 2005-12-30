function [rmean,rcov] = tst_match(m1,m2,m2in1)

%  res = plot_cell(m1,m2,m2in1)
% [rmean,rcov] = tstSample (m1,m2,m2in1);
 [rmean,rcov] = estimatePose(m1,m2,m2in1);
 
function [rmean, rcov] = estimatePose(m1,m2,m2in1)
  x0  = m2in1(1:3);
  P0  = eye(3);

  w0  = match_maps(m1, translate_obs(m2,m2in1))
  
  ind_obs_cov = [3 4; 5 6];

  % Compute x0,P0
  Saa = (3*pi/180/3)^2;
  
  m2_ = translate_obs(m2(1,:),[0 0 m2in1(3)]);
  m1_ = m1(1,:);
  Z1  = m1_(ind_obs_cov);
  Z2  = m2_(ind_obs_cov);
  
  P0  = [ Z1+Z2, [0;0]; 0 0 Saa];
  x0  = [m1_(1:2)' - m2_(1:2)'; m2in1(3)];

  x = x0;
  P = P0;
  
  for i = 2: size(m1,1)
    m2_ = translate_obs(m2(i,:), x);
    m1_ = m1(i,:);

    Z1  = m1_(ind_obs_cov);
    Z2  = m2_(ind_obs_cov);
    Z   = Z1 + Z2;
    
    ca  = cos(x(3));
    sa  = sin(x(3));
    
    H   = [ -ca , -sa,  m2_(2);
	     sa , -ca, -m2_(1)];

    S   = H*P*H' + Z;
    W   = P*H'*inv(S);
    
    innov = (m1_(1:2)' - m2_(1:2)');
    correction = W*innov;
    
    x   = x + correction;
    P   = P - W*S*W';
  end
  
  rmean = x;
  rcov  = P;
  w1  = match_maps(m1, translate_obs(m2,x))
  

 
function tstSample2(m1,m2,m2in1)
  NP = 100;
  nmap = size(m1,1); 
  
  M1 = zeros(nmap,2,NP);
 
function [rmean,rcov] = tstSample(m1,m2,m2in1)
 
  NP = 10;
  DX = .5;
  DY = .5;
  DA = 1*pi/180;
  
  r = rand(NP,3);
  r(:,1) = m2in1(1) + (2*DX*r(:,1) - DX);
  r(:,2) = m2in1(2) + (2*DY*r(:,2) - DY);
  r(:,3) = m2in1(3) + (2*DA*r(:,3) - DA);

  for i = 1:NP
    m2_ = translate_obs(m2,r(i,:));
    w(i) = match_maps(m1,m2_);
  end
  
  plot_robot(r,'g.-',0.1);
  
  w = w./sum(w);
  [rmean,rcov] = compute_cov(r,w');
  
  hold on
  plot_odo(rmean,rcov,'ror-r-r-r-',1.0);axis equal;
  hold off
  
  M = 2;
  A = 3;
  
  smpl = zeros(0,3);
  smpl_w = zeros(0,1);
  w = zeros(NP,1);
  NP = 100;

  for k =1:10
    [r,wg] = sample_gauss(rmean,rcov*A,NP);
    for i = 1:NP
      m2_ = translate_obs(m2,r(i,:));
      w(i) = match_maps(m1,m2_);
    end

    wg = M*wg./sum(wg);
    w_norm = w./sum(w);
    
    subplot(2,1,1)
    plot(w_norm,'r.-'); 
    hold on; 
    plot(wg,'g.-');

    ibad = find(wg < w_norm);
    if ~isempty(ibad)
      plot(ibad,w_norm(ibad),'bs');
    end
    hold off;
    
    
    u = rand(NP,1);
    W = w_norm./(wg);
    ind = find(u < W);
    
    subplot(2,1,2);
    plot_robot(r,'g.-',0.1);
    hold on
    length(ind)
    smpl = [smpl;r(ind,:)];
    smpl_w = [smpl_w;w(ind)];
    
    plot_robot(smpl,'bo-',0.02);
    
    rcov0 = rcov;
    rmean0 = rmean;
    
    [rmean,rcov] = compute_cov(smpl,smpl_w/sum(smpl_w))
    
    plot_odo(rmean,rcov,'ror-r-r-r-',1.0);axis equal;
    mplot(m1,'ro','r-');

    m2_ = translate_obs(m2,rmean);
    
    mplot(m2_,'gx','g-');
    hold off

    max(abs(rmean - rmean0))
    max(max(abs(rcov - rcov0)))
    
    pause(1);
  end
  
  

function [m,cov] = compute_cov(robot,w)

  m = sum(robot.*[w,w,w]);
  cov = zeros(3,3);
  
  dx = robot(:,1) - m(1);
  dy = robot(:,2) - m(2);
  da = angle_diff(robot(:,3),m(3));
  
  cov(1,1) = sum(dx.*dx.*w);
  cov(1,2) = sum(dx.*dy.*w);
  cov(1,3) = sum(dx.*da.*w);
  cov(2,2) = sum(dy.*dy.*w);
  cov(2,3) = sum(dy.*da.*w);
  cov(3,3) = sum(da.*da.*w);
  
  cov(2,1) = cov(1,2);
  cov(3,1) = cov(1,3);
  cov(3,2) = cov(2,3);
  

  

function [vol] = computeVolume(m1,m2,m2in1)
  DX = 1;
  DY = 1;
  DA = 30*pi/180;
  
  bmax = m2in1 + [DX,DY,DA];
  bmin = m2in1 - [DX,DY,DA];
  
  vol = triplequad(@aux1,bmin(1),bmax(1),bmin(2),bmax(2), ...
		   0, 2*pi, 1.e-3,[],m1,m2)


function [res,robot,w] = plot_cell(m1,m2,m2in1)

  DX = 0.5;
  DY = 0.5;
  DA = 1*pi/180;
  
  nx = 30;
  ny = 30;
  na = 10;
  
  x = linspace(m2in1(1) - DX, m2in1(1) + DX, nx);
  y = linspace(m2in1(2) - DY, m2in1(2) + DY, ny);
  a = linspace(m2in1(3) - DA, m2in1(3) + DA, na);
  
  rot = m2in1(3);
  res = zeros(nx,ny,na);
  
  nr = 0;
  
  for ia = 1:na
    for ix = 1:nx
      for iy = 1:ny
	nr = nr + 1;
	robot(nr,:) = [x(ix), y(iy), a(ia)];
	m2_ = translate_obs(m2,robot(nr,:));
	res(ix,iy,ia) = match_maps(m1,m2_);
	w(nr) = res(ix,iy,ia);
      end
    end
  end
  
  vol = sum(sum(sum(res)))*(x(2)-x(1))*(y(2)-y(1))*(a(2)-a(1))
  

function w = aux1(x,y,z,m1,m2)
  nx = length(x);
  
  for i = 1:nx
    robot = [x(i),y,z];
    m2_ = translate_obs(m2,robot);
    w(i) = match_maps(m1,m2_);
  end
  
  

function w = match_maps(m1,m2)
  n = size(m1,1);
  
  logW = 0;
  
  for i = 1:n
    logW = logW + matchLog(m1(i,1:2)',[m1(i,3:4); m1(i,5:6)], ... 
			   m2(i,1:2)',[m2(i,3:4); m2(i,5:6)]);
  end
  
  w = exp(logW);
  
function w = matchLog(m1,K1,m2,K2);

  K1_inv = inv(K1);
  K2_inv = inv(K2);
  
  C = inv(K1_inv + K2_inv);
  B = K1_inv*C*K2_inv;
  
  m = m1-m2;
  n = 2;

  w = -n*0.5*log(2*pi) + 0.5*log(det(C)/(det(K1)*det(K2))) ...
      -0.5*m'*B*m;
  
  