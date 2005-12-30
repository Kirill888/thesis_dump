function plot_agent(p,obs0,map,step)

  obs = obs0(find(obs0(:,7) == step),1:6);
  odo = p(step).p(:,4:6);
  odo(:,3) = odo(:,3)*pi/180;
  w   = p(step).p(:,7);
  
  plot_robot(odo,'g.-');
  hold on
  mplot(map.map,'ro','r-');
  
  [v,i] = max(w);
  
  obs = translate_obs(obs,odo(i,:));
  mplot(obs,'g.','g-');
  plot_robot(odo(i,:),'ro-');