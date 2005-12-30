function [hobs, hmap, hodo] = pplot(base , ind, obs_all, ...
                              style_obs  , style_obs_ellipse, ...
                              style_map  , style_map_ellipse, ...
			      style_robot, r)
  
%function [hobs, hmap, hodo] = pplot(base, ind, obs_all, ...
%                             style_obs, style_obs_ellipse, ...
%                             style_map, style_map_ellipse, ...
%		              style_robot,robot_r)
  
  format_odo = '%s_p%03d.odo';

  fodo = sprintf(format_odo,base,ind);

  odo  = load(fodo);

  [hobs hodo] = mplot(obs_all, style_obs, style_obs_ellipse, ...
                      odo, style_robot, r);
  
  
  if ~isempty(style_map)
     format_map = '%s_p%03d.map';
     fmap = sprintf(format_map,base,ind);
     map  = load(fmap);
     hmap = mplot(map, style_map, style_map_ellipse);
  else
     hmap = [];
  end
  
  if nargout == 0
    clear hobs hmap hodo
  end
