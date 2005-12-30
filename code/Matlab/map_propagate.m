function map = map_propagate(mm,odo)
%  function map = map_propagate(mm,odo)
  
  map = mm;
  
  map.map = propagate_obs(map.map,odo);
  map.odo = translate_odo(map.odo,odo);
  
  if isfield(map,'obs')
    map.obs = propagate_obs(map.obs,odo);
  end
  
