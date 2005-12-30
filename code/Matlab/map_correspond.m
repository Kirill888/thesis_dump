function [corr, xyr1, xyr2] = map_correspond(m1, m2, do_plot)
%  function [corr, xyr1, xyr2] = map_correspond(m1, m2)

  FILE_NAME_IN  = '/tmp/graph_in';
  FILE_NAME_OUT = '/tmp/graph_out.m';
  GRAPH_EXEC    = '/home/kirill/SW/Graph/graph 0';
  
  %Run graph matching program
  map_out(m1.map, m2.map, FILE_NAME_IN);
  tic;
  system([GRAPH_EXEC ' < ' FILE_NAME_IN ' > ' FILE_NAME_OUT]); 
  t = toc
  
  %Load graph matching results
  run(FILE_NAME_OUT);
  
  if isempty(permut)
    display('No solution was found');
    corr = [];
    xyr1 = [];
    xyr2 = [];
    return 
  end
  
  corr = permut(size(permut,1),4:size(permut,2));
  i1   = find(corr > 0);
  i2   = corr(i1);
  
  g1 = graph_make(g1,g1_p);
  g2 = graph_make(g2,g2_p);

  g1_ = graph_ind(g1,i1);
  g2_ = graph_ind(g2,i2);

  [xyr1,xyr2] = graph_allign(g1_,g2_);
  
  %Somehow angle notation is different.
  xyr1(3) = -xyr1(3);
  xyr2(3) = -xyr2(3);
  
  if nargin > 2 & do_plot
    m2_ = map_translate(m2, xyr1);
    
    if 0
      obs1_style = 'g.g:';
      obs2_style = 'c.c:';
    else
      obs1_style = '';
      obs2_style = '';
    end
    
    plot_map(m1 ,'ror-', obs1_style,'ro-',0.1);
    hold on
    plot_map(m2_,'bsb-', obs2_style,'bs-',0.1);
  end
  

  
  
function map_out(m1,m2,file_name)
%function map_out(m1,m2,file_name)

  fid = fopen(file_name,'w');

  if fid < 0
    error('Can''t open file');
  end

  map_out2(fid,m1);
  fprintf(fid,'#\n');

  map_out2(fid,m2);
  fprintf(fid,'#\n');

  fclose(fid);


function map_out2(fid,m)
  for i=1:size(m,1)
    fprintf(fid,' %+10.6f', m(i,:));
    fprintf(fid,'\n');
  end

  