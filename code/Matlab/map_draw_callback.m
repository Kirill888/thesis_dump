function map_draw_callback(agents,nagents,scanId)
  global maps;
  global frame;
  global lastMap;
  global msgPostTime;
  global frame_skip;

  global print_options;
  global print_mask;
  
  global maps_h;
  global agent_h;
  global msg_h;

  if mod(frame,1+frame_skip) ~= 0
    frame = frame + 1;
    return
  end
  
  delete(agent_h);

  agent = find_best_agent(agents,nagents);
  
  cmap = agent.current_map;
  ag_id = agent.id;
  
  maps{cmap} = agent.map;
  
  map_ref = adjust_map_frame(agent.map_ref);
  
  for i = 1:size(map_ref,1)
    if i <= size(maps_h,2)
      delete(maps_h{i})
    end
    
    maps_h{i} = plot_map(maps{i}, map_ref(i,:),i, i == cmap);
  end
  
  
  agent_h = plot_agent(agent,map_ref(cmap,:),scanId);
  h = text(15,9,sprintf('Current Map: %02d [%04d]',cmap,scanId));
  agent_h = [agent_h;h];
  
  if lastMap > 0 
    if lastMap > cmap + 1
      %Loop closing
      lastMap = size(map_ref,1);
      h = text(5,4,sprintf('Loop Closed: %02d->%02d',lastMap, ...
			   cmap));
      set(h,'FontSize',14);
      set(h,'Color','r');
      
      msg_h = h;
      msgPostTime = frame;
    end
  end
  
  if ~isempty(msg_h) & ...
      frame - msgPostTime > 5*25
    delete(msg_h);
    msg_h = [];
  end
  
  if ~isempty(print_mask)
    shot_id = frame./(frame_skip+1);
    file = sprintf(print_mask,shot_id)
    print(print_options, file);
  end
  
  
  drawnow
  frame = frame + 1;
  
  lastMap = cmap;

  
  
  %pause(0.01)

function agent = find_best_agent(agents,n)
  for i = 1:n
    if ~isempty(agents(i).odo)
      w(i) = sum(agents(i).odo(:,4));
    else
      w(i) = 0;
    end
  end
  [v,i] = max(w);
  agent = agents(i);
  
function ref = adjust_map_frame(ref0)
  ref = ref0;
  ref(:,1) = ref0(:,1) - ref0(1,1);
  ref(:,2) = ref0(:,2) - ref0(1,2);
  
  ref = translate_odo(ref,[0 0 -ref0(1,3)]);

function h = plot_agent(agent,pose,scanId)
  global obs0;
  
  if ~isempty(agent.map)
    map = translate_obs(agent.map, pose);
    mapx = map(:,1);
    mapy = map(:,2);
  else
    mapx = [];
    mapy = [];
  end
  
  odo = translate_odo(agent.odo, pose);
  odox = odo(:,1);
  odoy = odo(:,2);

  ind = find(obs0(:,7) == scanId);
  if ~isempty(ind)
    [v,i] = max(agent.odo(:,4));
    bestOdo = translate_odo(agent.odo(i,1:3),pose);
    obs = translate_obs(obs0(ind,1:6),bestOdo);
    
    lx = zeros(1,size(obs,1));
    ly = lx;
    
    for i = 1:size(obs,1)
      lx(2*(i-1)+1) = bestOdo(1);
      lx(2*(i-1)+2) = obs(i,1);

      ly(2*(i-1)+1) = bestOdo(2);
      ly(2*(i-1)+2) = obs(i,2);
    end
    
    h = line(lx,ly);
    set(h,'Color','r');
    set(h,'LineWidth',2);
  else
    h = [];
  end
  
  h2 = plot(mapx, mapy, 'r.' ...
	   ,odox,odoy, 'g.');
  h = [h;h2];
  

function h = plot_map(map,pose,id,current)

if ~isempty(map)
  map = translate_obs(map,pose);

  style = {'bo','bs','b^','b*','bd'};
  sid = mod(id,size(style,2)) + 1;
  
  h = plot(map(:,1), map(:,2), style{sid});
  str = sprintf('%02d',id);
  h2 = text(pose(1), pose(2), str);
  if current
    set(h2,'Color','r');
  else
    set(h2,'Color','b');
  end
  
  h = [h h2];
else
  h = [];
end










