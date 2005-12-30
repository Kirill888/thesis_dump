function viewLsr(las)

dttt = 0.2 ;
figu = 1 ;

global AAr; 
AAr = linspace(0,pi,361);

L = size(las,1);
Time = las(:,1); 
CAAA = cos(AAr);
SAAA = sin(AAr);

global pCircles ;
nc = 9 ; aaa = [0:nc]*2*pi/nc ; pCircles = [ cos(aaa); sin(aaa) ] ;  


figure(figu) ;clf ; 
set(figu,'DoubleBuffer','on','WindowButtonDownFcn','pause');
zoom on ;
hhh =plot(0,0,'.-','erasemode','xor') ;   %laser
hold on;
hhh2=plot(0,0,'ro','erasemode','xor') ;  % landmarks centers
hhh3=plot(0,0,'g','erasemode','xor') ;   % approx. landm. circles
axis([-80,80,0,125]); axis equal
hold off ;

for i=1:L,
  RR = las(i,2:362);
  xra=detectTrees(RR);
  
  if ~isempty(xra)
    DibuTrees(xl,yl,xra(3,:),hhh3) ;
    xl = xra(1,:).*cos(xra(2,:)) ;
    yl = xra(1,:).*sin(xra(2,:)) ;
    set(hhh2,'XData',xl,'YData',yl) ;	
  end
  
  ii2 = find(RR<1000) ;
  xx = RR(ii2).*CAAA(ii2) ;
  yy = RR(ii2).*SAAA(ii2) ;
  xx = [xx, xx(1)]; yy = [yy, yy(1)];
  set(hhh,'XData',xx,'YData',yy) ;	
  
  title(sprintf('k = %d  Num Trees %02d',i,size(xra,2)));
  pause(dttt) ;
end;
return ;

% --------------------------------------------


function DibuTrees(xl,yl,rl,hdl)
global pCircles ;
xyCi = pCircles ;
nc = size(xyCi) ; nc = nc(2) ;
nl = length(xl) ;
u=1 ;
z=1 ;
xyAllC = zeros(2,nc*nl+nl) ;
xxx    = zeros(2,nl) ;

for i=1:nl,
	Ri=rl(i)*0.5 ;
	xyCi(1,:)= pCircles(1,:)*Ri + xl(i) ;
	xyCi(2,:)= pCircles(2,:)*Ri + yl(i) ;
	xyAllC(:,u:u+nc-1)=xyCi ;
	xxx(:,nl-i+1) = xyCi(:,1) ;
	u=u+nc ;
end ;
xyAllC(:,u:end) = xxx ;
set(hdl,'XData',xyAllC(1,:),'YData',xyAllC(2,:)) ;	
return ;


% --------------------------------------------
	
