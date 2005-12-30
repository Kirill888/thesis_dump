function v = vol(a,Paa,b,Pbb)
  
  a1 = gauss2a(a,Paa);
  a2 = gauss2a(b,Pbb);
  aa = a1 + a2;
  v = volume(aa);
