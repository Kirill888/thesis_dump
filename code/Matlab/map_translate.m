function map = map_translate(map,odo)

  map.odo = translate_odo(map.odo, odo);

  map.map = translate_obs(map.map, odo);

  if isfield(map,'obs')
    map.obs = translate_obs(map.obs, odo);
  end
  
  if isfield(map,'las')
    map.las = translate_obs(map.las, odo);
  end