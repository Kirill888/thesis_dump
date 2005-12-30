function store(odo,obs,las,base)

t_obs = unique(obs(:,1));
t_odo = odo(:,1);

n_odo = size(t_odo,1);
n_obs = size(t_obs,1);

man = [t_odo, ones(n_odo,1);
       t_obs, ones(n_obs,1)*2];

man = sortrows(man,1);

fd = fopen([base '.man'],'w');
fprintf(fd,'%7.2f %d\n',man');
fclose(fd);

fd = fopen([base '.odo'],'w');
fprintf(fd,'%7.2f %e %e\n',odo');
fclose(fd);

fd = fopen([base '.obs'],'w');
fprintf(fd,'%7.2f %2d %e %e %e\n',obs');
fclose(fd);

save([base '.las'],'-ASCII','las');

