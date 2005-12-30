function save_templates(t,filename)

f = fopen(filename,'w');

t = round(128*t)+128;
ii = find(t(:) > 255);
t(ii) = 255;
ii = find(t(:) < 0);
t(ii) = 0;

for i = 1:size(t,1)
  fprintf(f,'%02X',t(i,:));
  fprintf(f,'\n');
end

fclose(f);
