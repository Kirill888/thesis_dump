function [c,p] = findCorner(x,y)
% function [c,p] = findCorner(x,y)
%
%
%

MIN_PTS_PER_LINE = 5;
D_SIGMA2 = 0.05*0.05;

np = length(x);

if np < 2*MIN_PTS_PER_LINE
  error('','Need at least %d points',MIN_PTS_PER_LINE*2);
end

for i = MIN_PTS_PER_LINE:(np-MIN_PTS_PER_LINE)
  [l1,err1] = fit_line(x(1:i),y(1:i));
  [l2,err2] = fit_line(x((i+1):end), y((i+1):end));
  
  err(i) = sum(err1.*err1) + sum(err2.*err2);
end


[p,c] = min(err(MIN_PTS_PER_LINE:end));
c = c + MIN_PTS_PER_LINE;
err(1:MIN_PTS_PER_LINE) = max(err);

figure(1);
subplot(2,1,1);
plot(-0.5*err);

subplot(2,1,2);
plot(x,y,'b.-',x(c),y(c),'rs');
axis equal


