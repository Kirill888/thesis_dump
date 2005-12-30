function tst1(x,y, a,Paa, b, Pbb)
  
  figure
  
  aux_display(x,y,a,Paa,b,Pbb)
  
  b = collect_points('go',axis,1);
  while ~isempty(b)
    aux_display(x,y,a,Paa,b,Pbb)
    b = collect_points('go',axis,1);
  end
  
  
  
function aux_display(x,y, a,Paa, b, Pbb)
  
  cla;
  
  vv = gauss_prod(x, y, a,Paa, b, Pbb);
  
  pcolor(x,y,vv);
  hold on
  
  plot_map_g(a,Paa,'ro','r-');
  plot_map_g(b,Pbb,'go','g-');
  
  w = 0:0.05:1;
  
  [c Pcc] = cov_intersect(a,Paa,b,Pbb,0.5);
  plot_map_g(c,Pcc,'ms','m-');

  
  axis equal
  xlabel('x'); ylabel('y');
  
  
  