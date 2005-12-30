function o = join(files)
% function join(files)

load(files{1});
o = trim(out);

for i = 2:length(files)
  load(files{i});
  o = [o; trim(out)];
end




function trimmed = trim(in)

ii = find(in(:,1) ~= 0);
trimmed = in(ii,:);

