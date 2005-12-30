function [ll,pl] = lookForCorners(scan, NFIT)
% function lookForCorners(las_scan, NFIT)
%  ll -lines
%  pl - error of the line
%

%


if nargin < 2
  NFIT = 10;
end

avgErrPerPoint = 0.005;

goodLine = -avgErrPerPoint*avgErrPerPoint*NFIT;
badLine  = 1.2*goodLine;

o = linspace(0,pi,361);
x = scan.*cos(o);
y = scan.*sin(o);

fit_win = 0:(NFIT-1);

np = length(x);
nl = np - NFIT + 1;

ll = zeros(nl,3);
pl = zeros(nl,1);

for i = 1:nl
  ii = fit_win + i;

  [ll(i,:),err] = fit_line(x(ii),y(ii));
  
  pl(i) = sum(err.*err);  
end

ii1 = 1:(nl-NFIT);
ii2 = (NFIT+1):nl;

ind_good = find(-pl > goodLine);
dii = diff(ind_good);
ii  = find(dii > 1);
ii1 = ind_good(ii);
ii2 = ind_good(ii+dii(ii+1));


figure(1);
%subplot(2,1,1);
h = plot(x,y,'b.-', ...
     x(ii1+NFIT-1),y(ii1+NFIT-1),'ro', ...
     x(ii2),y(ii2),'rs');

set(h(2:3),'MarkerSize',6,'LineWidth',2);

ind_t = 20:20:361;
text(x(ind_t),y(ind_t),num2str(ind_t'));

%title('Scan');
axis equal


%subplot(2,1,2);
figure(2)
xp = 1:length(pl);
lx = [xp(ii1);xp(ii2)];
ly = [-pl(ii1)';-pl(ii2)'];

h = plot(xp,-pl,'b.-', lx,ly,'ro-',...
     [xp([1,end])], goodLine.*ones(2,1) , 'r-');

set(h(2:end),'LineWidth',2);
%title('Average distance squared negated');
axis([1,361,2*badLine,0]);
ylabel('Negated point-line error (m^2)');
xlabel('Line index');
print -depsc line_point_error.eps


figure(1)
print -depsc scan.eps



if 0
pp = -0.5*(pl(ii1) + pl(ii2));
ipeaks = find_peak_after_valley(pp,badLine,goodLine);

cos_a = ll(ii1,1).*ll(ii2,1) + ll(ii1,2).*ll(ii2,2);
a = acos(cos_a)*180/pi;

figure(1);
subplot(2,1,1);
plot(x,y,'b.-',x(ipeaks + NFIT - 1),y(ipeaks + NFIT - 1),'ro');
text(x(ipeaks + NFIT - 1),y(ipeaks + NFIT - 1),num2str([1: ...
		    length(ipeaks)]'));
title('Scan');
axis equal

subplot(2,1,2);
xp = 1:length(pp);
plot(xp,pp,'b.-',xp(ipeaks),pp(ipeaks),'ro', ...
     xp, goodLine, 'r-', badLine,'g-');
text(xp(ipeaks),pp(ipeaks),num2str([1:length(ipeaks)]'));
title('Average distance squared negated');
axis([1,361,2*badLine,0]);

%subplot(2,2,4)
%plot(xp, a, 'b.-', ...
%     xp(ipeaks),a(ipeaks),'ro');
%text(xp(ipeaks),a(ipeaks),num2str([1:length(ipeaks)]'));
%title('Line slope in degrees');
end


function ind = find_peak_after_valley(x,minValley,minPeak)
 nx = length(x);

 i = 2;
 ind_count = 0;
 ind = [];
 
 while i < nx
   
   %Search for valley
   if x(i) < minValley && ...
	 x(i) < x(i-1) && ...
	 x(i) < x(i+1)
     %Search for peak
     
     i = i + 1;
     
     while i < nx
      if x(i) > minPeak  && ...
         x(i) > x(i - 1) && ...
         x(i) > x(i + 1)
         
	 ind_count = ind_count + 1;
	 ind(ind_count) = i;
	 break;
      end
      i = i + 1;
     end
   
   end
   
   i = i + 1;
 end
 


function ind = find_peaks(x,min_val)

   if x(1) > min_val && x(1) > x(2)
     ind = 1;
     ind_count = 1;
   else
     ind = [];
     ind_count = 0;
   end
   nx = length(x);

   for i = 2:(nx -1);
      if x(i) > min_val  && ...
         x(i) > x(i - 1) && ...
         x(i) > x(i + 1)
         
	 ind_count = ind_count + 1;
	 ind(ind_count) = i;
      end
   end

   if x(nx) > min_val && x(nx) > x(nx -1)
      ind(ind_count + 1) = nx;
   end
