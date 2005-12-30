function v = gauss_prod(x, y, a, Paa, b, Pbb)
%function v = gauss_prod(x, y, a, Paa, b, Pbb)
  
  v1 = gauss_d(x,y,a,Paa);
  v2 = gauss_d(x,y,b,Pbb);
  
  v = v1.*v2;
  