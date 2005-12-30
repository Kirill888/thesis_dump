function map = translate_map(map,odo)
  map.odo = translate_odo(map.odo, odo);
  map.map = translate_obs(map.map, odo);
