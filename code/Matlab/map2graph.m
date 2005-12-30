function g = map2graph(m)
%function g = map2graph(m)
 
  if size(m,1) == 1
    n = size(m,2);
    
    for i = 1:n
      g(i) = aux1(m(i).map);
    end
    
  else
    g = aux(m);
  end
  

function g = aux1(m)
  g.n  = size(m,1);
  g.xy = m(:,1:2);
  g.e  = graph_edges(g.xy);
  
  for i = 1:g.n
    g.p(1:2,1:2,i) = [m(i,3:4); m(i,5:6)];
  end
  