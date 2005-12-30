function MIT_play(fname)
% function MIT_play(fname)
%

%

f = fopen(fname,'r');

fig = figure(1);
set(fig,'DoubleBuffer','on');

hl = plot(0,0,'b.-')

axis([-70 70 -70 70]);
axis equal

s = fgets(f);
numLas = 360;

a = linspace(0,pi,numLas);
CA = cos(a);
SA = sin(a);

while s ~= -1
  if strncmp('FLASER', s, 5)
    ll = sscanf(s(11:end),'%lf',[1,numLas]);
    
    lx = ll.*CA;
    ly = ll.*SA;
    
    set(hl,'XData',lx,'YData',ly);
    drawnow;
    
    pause(0.1);
  end

  s = fgets(f);
end

