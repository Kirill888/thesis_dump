function v = match_prob(a, Paa, b, Pbb, ax)
%function v = match_prob(a, Paa, b, Pbb, ax)

  aux1(a,Paa,b,Pbb);
  
  v = dblquad( @aux1, ax(1), ax(2), ax(3), ax(4));
  
  
  


function v = aux1(a1, a2, a3, a4)
  persistent a Paa b Pbb
  
  if nargin == 4
    a   = a1;
    Paa = a2;
    b   = a3;
    Pbb = a4;
  else
    v1 = gauss_d(a1, a2, a, Paa);
    v2 = gauss_d(a1, a2, b, Pbb);
    v  = v1.*v2;
  end
  