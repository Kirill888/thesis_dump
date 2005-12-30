function [c,Pcc, N, md2] = cov_tst(a, Paa, b, Pbb)
  
  w = 0:0.01:1;
  
  [c, Pcc, N] = cov_intersect(a, Paa, b, Pbb, w);
  
  figure
  
  
  subplot(1,2,1);
  hold on

  plot_map_g(c, Pcc, 'c.', 'c:');
  plot_map_g(a, Paa, 'ro', 'r-');  
  plot_map_g(b, Pbb, 'bo', 'b-');
  
  [v,i] = min(N);
  
  plot_map_g(c(i(1),:), Pcc(:,:,i(1)), 'm.', 'm-');
  plot_map_g(c(i(2),:), Pcc(:,:,i(2)), 'g*', 'g-');
  
  axis equal
  
  subplot(2,2,2)
  

  plot(w,N(:,1), 'b.-', w(i(1)), N(i(1),1),'bs');
  hold on
  plot(w,N(:,2), 'r.-', w(i(2)), N(i(2),2),'ro');

  
  xlabel('w');
  ylabel('Norm(cov)');
  title('Norm(cov)');


  
  md2_a = mahalanobis2(a, Paa, c)';
  md2_b = mahalanobis2(b, Pbb, c)';
  
  md2 = [ md2_a md2_b];
  sum_md2 = sum(md2,2);
  
  in = i;
  
  [v,i] = min(sum_md2);
  
  subplot(2,2,4);
  
  plot(w, md2(:,1),'r.-', ...
       w, md2(:,2),'b.-', ...
       w, sum_md2, 'm.-', ...
       w(i), sum_md2(i), 'ro');

  title(sprintf(...
      ['MD^2, (best match at %g = %g, %g)\n' ...
       '(norm1 match at %g = %g,%g)\n' ...
       '(norm2 match at %g = %g,%g)'] ...
	, w(i), sum_md2(i), exp(-0.5*sum_md2(i)) ...
	, w(in(1)), sum_md2(in(1)), exp(-0.5*sum_md2(in(1))) ...
	, w(in(2)), sum_md2(in(2)), exp(-0.5*sum_md2(in(2))) ...
      ));
  xlabel('w');
  ylabel('Mahalanobis^2');

  subplot(1,2,1);
  plot_map_g(c(i,:), Pcc(:,:,i), 'ks', 'k-');
 
