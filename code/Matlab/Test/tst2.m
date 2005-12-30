function tst2(a,Paa,Pbb)
  
  figure;

  %Compute x,y
  NPX = 100;
  NPY = 100;
  d = 3.1*sqrt(max([eigs(Paa);eigs(Pbb)]))
  x = linspace(a(1)-d,a(1)+d,NPX);
  y = linspace(a(2)-d,a(2)+d,NPY);

% $$$   vv = match_hist(x,y,a,Paa,Pbb, @match1);
% $$$   subplot(2,2,1);
% $$$   aux_plot(x,y,a,Paa,Pbb,vv);
% $$$   axis equal
% $$$   title('Match A->B');
% $$$   compute_volume(x,y,vv)
% $$$  
% $$$   vv = match_hist(x,y,a,Paa,Pbb, @match2);
% $$$   v  =   compute_volume(x,y,vv)
% $$$   vv = vv./v;
% $$$   
% $$$   subplot(2,2,2);
% $$$   aux_plot(x,y,a,Paa,Pbb,vv);
% $$$   axis equal
% $$$   title('Match B->A');
% $$$ 
% $$$    
% $$$   vv = match_hist(x,y,a,Paa,Pbb, @match3);
% $$$   v  =   compute_volume(x,y,vv)
% $$$   vv = vv./v;
% $$$   subplot(2,2,3);
% $$$   aux_plot(x,y,a,Paa,Pbb,vv);
% $$$   axis equal
% $$$   title('Match A->C and B->C (c = cov.int. at 0.5)');

  
%  vv = match_hist(x,y,a,Paa,Pbb, @match4);
  vv = match_hist(x ,y,a,Paa,Pbb, @match5);
  v  =   compute_volume(x,y,vv)

%  subplot(2,2,4);

  aux_plot(x,y,a,Paa,Pbb,vv);
  axis equal
%  title('Match A->C and B->C (c = cov.int. at min norm2)');
  title('Volume of the product');

  
  orient landscape
  colormap('gray');
  cmap = colormap;
  colormap(cmap(length(cmap):-1:1,:));
  
  figure; mesh(x,y,vv);

function aux_plot(x,y, a,Paa,Pbb,vv)
  pcolor(x,y,vv);
  hold on
  
  plot_map_g(a,Paa,'ro','r-');
  plot_map_g(a,Pbb,'go','g-');

function v = compute_volume(x,y,vv)
  dx = diff(x);
  dy = diff(y);
  
  dx = [dx(1) dx];
  dy = [dy(1) dy];
  
  dd = dy'*dx;
  
  v = sum(sum(dd.*vv));

function v = match1(a,Paa,b,Pbb)
  % - MD2 a => b
    v = exp(-mahalanobis2(a,Paa,b));

function v = match2(a,Paa,b,Pbb)
  % -MD2 b => a
    v = exp(- mahalanobis2(b,Pbb,a));

function v = match3(a,Paa,b,Pbb)
  % -MD2 (a => c + b =>c) where c is center of cov.intersection w = 0.5
  [c,Pcc] = cov_intersect(a,Paa,b,Pbb,0.5);
  
  v = exp(-mahalanobis2(a,Paa,c) - mahalanobis2(b,Pbb,c));

function v = match4(a,Paa,b,Pbb)
  % -MD2 (a => c + b => c) where c is center of cov.intersection
  % with minimal norm2
    
  w = 0:0.05:1;
  [c,Pcc,N] = cov_intersect(a,Paa,b,Pbb,w);
  [v,i] = min(N(:,2));
  v = exp(-mahalanobis2(a,Paa,c(i,:)) - mahalanobis2(b,Pbb,c(i,:)));
  
  
function v = match5(a,Paa,b,Pbb)
if 0
  v =  dblquad('gauss_prod',-5,5,-5,5,[],[],a,Paa,b,Pbb);
else
  a1 = gauss2a(a,Paa);
  a2 = gauss2a(b,Pbb);
  aa = a1 + a2;
  v = volume(aa);

end






