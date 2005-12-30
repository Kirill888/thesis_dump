function [gmap] = load_map(base)
%function [gmap] = load_map(base)
%
%

%

  run([base '_links']);
  run([base '_mat']);

  odo(:,2:4) = las2odo(odo(:,2:4));
  
  fmaps = map_files;
  
  %Load maps
  nf = length(fmaps);
  
  for i = 1:nf
    disp(sprintf('Loading: %s',fmaps{i}));
    ii = find(odo(:,1) == i);
    
    if max(ii) > length(odo2scan)
       m(i) = mapLoad(fmaps{i}, odo(ii,2:4), []);
    else
       m(i) = mapLoad(fmaps{i},odo(ii,2:4), odo2scan(ii));
    end
  end

  gmap.links     = links;
  gmap.map_poses = map_poses;
  gmap.maps      = m;
  
  %Load Transition
  for i = 1:size(links,1)
    ii = find(transitions(:,1) == i);
    t.map1 = links(i,1);
    t.map2 = links(i,2);
    t.odo  = transitions(ii,2:4);
    t.w    = transitions(ii,5);
    
    tr(i) = t;
  end

  gmap.transitions = tr;
  gmap.odo = odo;
  
  if exist('obs0','var')
    gmap.obs0 = obs0;
  else
    gmap.obs0 = [];
  end
  
function odo = las2odo(las)
  X_LAS = 0.261;
  Y_LAS = 0.0;
  
  odo = las;
  ca = cos(las(:,3));
  sa = sin(las(:,3));
  
  odo(:,1) = las(:,1) - X_LAS*ca + Y_LAS*sa;
  odo(:,2) = las(:,2) - X_LAS*sa - Y_LAS*ca;
  
  
function m = mapLoad(fname, odo, odoScan)
  run(fname);
  
  if isempty(map)
    m.map = zeros(0,7);
  else
    m.map = map;
  end
  
  m.odo = odo;
  m.odoScan = odoScan;
  
  if exist('region','var')
    m.grid = region;
  end
  
  
  nmap = size(m.map,1);

  % If mapping to observation is stored as well
  if exist('map2obs','var')
    for i = 1:nmap
      iobs = find(map2obs(:,1) == i);
      m.map2obs{i} = map2obs(iobs,2);
    end
  end
  
% $$$ 
% $$$ function tr = transLoad(fname)
% $$$   run(fname);
% $$$   
% $$$   tr.map1 = imap(1) + 1;
% $$$   tr.map2 = imap(2) + 1;
% $$$   tr.odo  = transition(:,1:3);
% $$$   tr.w    = transition(:,4);