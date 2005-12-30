function plot_map(map,map_style, obs_style, odo_style, odo_r)
% function plot_map(map,map_style, obs_style, odo_style, odo_r)


  hold on


  if ~isempty(obs_style) & isfield(map,'obs')
    mplot(map.obs, obs_style(1:2), obs_style(3:length(obs_style)));
  end

  if ~isempty(map_style)
    mplot(map.map, map_style(1:2), map_style(3:length(map_style)));
    
    %SPAM:
    if size(map.map,2) == 14
      h = plot_corner_map(map.map,0.3);
      set(h,'Color',map_style(1),'LineWidth',2);
    end
  end

  if ~isempty(odo_style)
    plot_robot(map.odo,odo_style, odo_r);
  end
