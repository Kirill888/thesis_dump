function [man, odo, obs] = edge_save(obs,templates, out_dir)
% function edge_save(obs, templates, out_dir)
%  edge_save(out2,t1,'../RunX')
%

left_t  = load('left.time');
right_t = load('right.time');

odo     = load('odo.log');

nl = length(left_t);
nr = length(right_t);
nf  = min(nr,nl);

if nl ~= nr
  left_t  = left_t(1:nf);
  right_t = right_t(1:nf);
end

dt = left_t - right_t;
bad_frames = find(abs(dt) > 1/15);

if ~isempty(bad_frames) 
  disp('Dropping following frames:');
  disp(sprintf('%04d\n',bad_frames));

  %Remove frames with bad time stamps
  ii_bad = find2(obs(:,1), bad_frames);
  ii_good = setdiff(1:1:nf, ii_bad);
  obs = obs(ii_good,:);
end


frames = unique(obs(:,1));

T0 = min(left_t(frames(1)), right_t(frames(1)));
left_t   = left_t   - T0;
right_t  = right_t  - T0;
odo(:,1) = odo(:,1) - T0;



t_obs  = (left_t(frames) + right_t(frames))*0.5;

odo = extractSegment(odo,-1,t_obs(end)+1);



man = [ odo(:,1), 1*ones(size(odo,1),1)   ; ...
	t_obs   , 2*ones(size(t_obs,1),1) ];

man = sortrows(man,1);

%Prepare templates
tt = round(128*templates) + 128;
ii = find(tt > 255);
tt(ii) = 255;
ii = find(tt < 0);
tt(ii) = 0;

% Write stuff to files.
cur_dir = cd;

cd(out_dir);

f = fopen('obs.log','w');

for i = 1:size(obs,1)
  s = sprintf('%0X',tt(i,:));
  fprintf(f,'%3d  %3d %3d %.8e %3d %3d %s\n',obs(i,:),s);
end

fclose(f);

f = fopen('odo.log','w');
fprintf(f,'%.3f %7.3f %7.3f %.11e\n',odo');
fclose(f);

f = fopen('man.log','w');
fprintf(f,'%.3f %3d\n',man');
fclose(f);

cd(cur_dir);


function A = extractSegment(A0,t_start,t_end)
 i1 = find(A0(:,1) > t_start);
 i2 = find(A0(:,1) > t_end);
 
 if isempty(i2)
   A = A0(i1,:);
 else
   A = A0(i1(1):1:i2(1),:);
 end


