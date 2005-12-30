function print_fig(fig, fname)
%function print_fig(fig, fname)
%
%


  
  nf = length(fig);
  
  for i=1:nf 
    
    figure(fig(i));
    
    orient landscape
  
    if nargin > 1
      if size(fname,1) == nf
	print('-dpsc',fname{i});
      else
	print('-dpsc',sprintf(fname,i));
      end
    else
      print -dpsc
    end
    
  end
  