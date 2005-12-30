function m2 = map2d(mm)
% Project 3d map in to 2d

m2 = mm;

for i = 1:length(mm)
  m2(i).map = mm(i).map(:,[1 2 4 5 7 8]);
  m2(i).map_all = mm(i).map(:,[1 2 4 5 7 8]+1);
end
  