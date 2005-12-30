function edge_save(obs, odo, man, templates)
% function edge_save(obs, odo, man, templates)
%  edge_save(out2,odo,man,t1)
%

if iscell(templates)
  t = char(templates);
else
  t = templates;
end

f = fopen('obs.log','w');
for i = 1:size(obs,1)
  fprintf(f,'%3d  %3d %3d %.8e %3d %3d %s\n',obs(i,:),t(i,:));
end

fclose(f);

f = fopen('odo.log','w');
fprintf(f,'%.3f %.8e %.8e %.8e\n',odo');
fclose(f);

f = fopen('man.log','w');
fprintf(f,'%.3f %3d\n',man');
fclose(f);

