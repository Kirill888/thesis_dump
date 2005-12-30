function [obs,odo,man, t1] = edge_load(obs_file,t_file, odo_file)
%function [obs,odo,man,t1] = edge_load(obs_file,t_file, odo_file)

TEMPLATE_FILE = 'template1.txt'

if ~exist('t_file','var')
  t_file = 'timestamps.log';
end

if ~exist('odo_file','var')
  odo_file = 'odo.log';
end

if nargout > 3 && exist(TEMPLATE_FILE,'file')
  templates = textread(TEMPLATE_FILE,'%s\n');
else
  templates = [];
end

 t = load_timestamp(t_file);
 nt = size(t,1);
 dt = t(:,2) - t(:,1);
 bad_frames = find(dt > 0.1);

 odo = load(odo_file);
 obs = load(obs_file);
 
 %Adjust from 0 based indexing
 frames = obs(:,1) + 1;
 nf     = size(obs,1);

 %Delay between capture of left and right image
 dt = t(:,2) - t(:,1);
 bad_frames = find(dt > 0.1);

 maxFrame1 = length(t);
 maxFrame2 = frames(end);
 
 if maxFrame2 > maxFrame1
   disp(sprintf('Warning timestamp size does not match: %d %d', ...
		maxFrame1, maxFrame2));

   bad_frames = [bad_frames; [(maxFrame1+1):maxFrame2]'];
 end
 
 
 %Remove frames with bad time stamps
 ii_bad = find2(frames, bad_frames);
 ii_good = setdiff(1:1:nf, ii_bad);
 obs = obs(ii_good,:);
 
 if ~isempty(templates)
   t1 = templates(ii_good);
 else
   t1 = [];
 end
 

 disp(sprintf('Removing %d frames, %d/%d obs.' ...
	      ,length(bad_frames), length(ii_bad), nf));
 disp('Saving bad frames(1 based) to: badframes.txt');
 f = fopen('badframes.txt','w');
 fprintf(f,'%3d\n',bad_frames);
 fclose(f);


 %Initiate manager data
 tmp = unique(obs(:,1)+1);
 t_obs = t(tmp,1);
 nobs = length(t_obs);

  
 Tstart = t_obs(1) - 1.0;
 Tend   = t_obs(nobs) + 1.0;

 disp(sprintf('Tstart = %.3f',Tstart));
 disp(sprintf('Tend   = %.3f',Tend));
 
 odo = extractSegment(odo,Tstart,Tend);
  
 T0 = min([odo(1,1), t_obs(1)]);
 
 disp(sprintf('T0 = %.3f',T0));
 
 odo(:,1) = odo(:,1) - T0;
 t_obs = t_obs - T0;

 man = [  odo(:,1), 1*ones(size(odo,1),1); ...
	  t_obs,    2*ones(nobs,1);        ...
       ];
 
 man = sortrows(man,1);

 
function t = load_timestamp(file)
  t = textread(file,'%*s %f');
  nt = length(t);
  if mod(nt,2) ~= 0
    nt = nt - 1;
  end
  
  t = [t(1:2:nt), t(2:2:nt)];
  

function A = extractSegment(A0,t_start,t_end)
 i1 = find(A0(:,1) > t_start);
 i2 = find(A0(:,1) > t_end);
 
 if isempty(i2)
   A = A0(i1,:);
 else
   A = A0(i1(1):1:i2(1),:);
 end
 


