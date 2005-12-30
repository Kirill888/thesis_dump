function [obs,odo,las,man, ii_good] = vis_load2(points_file,t_file, ...
						odo_file, las_file)
%function [obs,odo,las,man,ii_good] = vis_load2(points_file,t_file, ...
%                                               odo_file, las_file)
%
%

%

if ~exist('t_file','var')
  t_file = 'TimeStamp.log';
end

if ~exist('odo_file','var')
  odo_file = 'odo.log';
end

if ~exist('las_file','var')
  las_file = [];
end

 if ~isempty(las_file)
    las = load(las_file);
 else
   las = [];
 end

 t = load_timestamp(t_file);
 nt = size(t,1);
 dt = t(:,2) - t(:,1);
 bad_frames = find(dt > 0.1);
 
 odo = load(odo_file);
 points = load(points_file);
 nf     = size(points,1);
 
 points(:,1) = points(:,1) + 1; %Adjust frames to 1 based.
 frames = points(:,1);

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
 points = points(ii_good,:);

 disp(sprintf('Removing %d frames, %d/%d obs.' ...
	      ,length(bad_frames), length(ii_bad), nf));
 
 %Convert point correspondences to obs
 %  note: this also removes bad correspondences
 [obs, igood] = vis_points2obs(points);
 
 ii_good = ii_good(igood);
 
 frames = unique(obs(:,1));

 t_obs = t(frames, 1);
 nobs = length(t_obs);

 Tstart = t_obs(1) - 1.0;
 Tend   = t_obs(nobs) + 1.0;

 disp(sprintf('Tstart = %.3f',Tstart));
 disp(sprintf('Tend   = %.3f',Tend));
 
 
 odo = extractSegment(odo,Tstart,Tend);
 T0 = min([odo(1,1), t_obs(1)]);
 
 if ~isempty(las)
   las = extractSegment(las,Tstart,Tend);
   T0 = min([T0, las(1,1)]);
 end
 

 disp(sprintf('T0 = %.3f',T0));
 
 odo(:,1) = odo(:,1) - T0;
 t_obs    = t_obs    - T0;
 
 man = [  odo(:,1), 1*ones(size(odo,1),1); ...
	  t_obs,    2*ones(nobs,1);        ...
       ];
 
 if ~isempty(las)
   las(:,1) = las(:,1) - T0;
   man = [man; las(:,1), 3*ones(size(las,1),1)];
 end
 
 man = sortrows(man,1);
 
 %Compute Uncertainty
 nf = size(obs,1);
 r = obs(:,2);
 sigma_r2 = square(0.2*r/3) + square(0.3/3);
 sigma_a2 = square(10.0*pi/180/3);
 sigma_b2 = square(10.0*pi/180/3);
 
 cov = zeros(nf,9);
 cov(:,1) = sigma_r2;
 cov(:,5) = sigma_b2;
 cov(:,9) = sigma_a2;
 
 obs = [obs, cov];



 
function aa = square(a)
  aa = a.*a;
 
function t = load_timestamp(file)
  t = textread(file,'%*s %f');
  nt = length(t);
  if mod(nt,2) ~= 0
    nt = nt - 1;
  end
  
  t = [t(1:2:nt),t(2:2:nt)];
  

function A = extractSegment(A0,t_start,t_end)
 i1 = find(A0(:,1) > t_start);
 i2 = find(A0(:,1) > t_end);
 
 if isempty(i2)
   A = A0(i1,:);
 else
   A = A0(i1(1):1:i2(1),:);
 end
