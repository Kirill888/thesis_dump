function map_draw_init(f_skip,pr_opts,pr_mask)
  global maps;
  global frame;
  global lastMap;
  global msgPostTime;
  global frame_skip;

  global maps_h;
  global agent_h;
  global msg_h;

  global print_options;
  global print_mask;


  maps    = [];
  frame   = 0;
  lastMap = -1;
  msgPostTime = -1;
  
  maps_h  = [];
  agent_h = [];
  msg_h   = [];
  
  if nargin > 0
    frame_skip = f_skip;
  else
    frame_skip = 0;
  end
  
  if nargin > 1 
    print_options = pr_opts;
    print_mask    = pr_mask;
  else
    print_options = [];
    print_mask = [];
  end
  
  figure
  axis equal
  axis tight
  axis([-1 40 -3 10]);
  hold on