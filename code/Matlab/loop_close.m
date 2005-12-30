 function loop_close(maps, ind_map1, ind_map2, np)
%function loop_close(maps, ind_map1, ind_map2)
 
  if nargin < 4
    np = 1000;
  end
  
 m1 = maps(ind_map1);
 m2 = maps(ind_map2);
 
 disp(sprintf('Finding map correspondences %d => %d',ind_map1, ind_map2));
 [corr, xyr1 xyr2] = map_correspond(m1, m2);
 
 if isempty(corr)
   display('No match');
   return
 end
 
 disp(' M1 => M2');
 for i = 1:length(corr)
   if corr(i) > 0
     disp(sprintf('%03d => %03d, %.3e %.3e => %.3e %.3e' ...
		  , i, corr(i) ...
		  ,m1.map(i,1), m1.map(i,2) ...
		  ,m2.map(corr(i),1), m2.map(corr(i),2)));
   else
     disp(sprintf('%03d => none', i));
   end     
 end
 
 disp(sprintf('Translation: %.3e %.3e %.3e', xyr1));
 
 m2_ = map_translate(m2, xyr1);
 disp(' M1 => M2');
 for i = 1:length(corr)
   if corr(i) > 0
     disp(sprintf('%03d => %03d, %.3e %.3e => %.3e %.3e' ...
		  , i, corr(i) ...
		  ,m1.map(i,1), m1.map(i,2) ...
		  ,m2_.map(corr(i),1), m2_.map(corr(i),2)));
   else
     disp(sprintf('%03d => none', i));
   end     
 end
 
 
 
 disp('Sampling Path');
 path = sample_path(maps, np, ind_map1, ind_map2);
 disp('done...');
 
 disp('Starting Plotting');
 
 i_last = size(path,2);
 i_last = i_last-2:i_last;
 
 odo_last = path(:, i_last);
 
 plot_robot(odo_last,'g.-',0.1);
 hold on
 
 plot_map(m1 ,'ror-','','ro-',0.2);
 plot_map(m2_,'bsb-','','bs-',0.2);

 ind = find(corr > 0);
 M1 = m1.map(ind,:);
 M2 = m2_.map(corr(ind), :);

% $$$  md2 = map_match(M1, M2);
% $$$  
% $$$  sqrt([md2 sum(md2,2)])
% $$$  
% $$$  exp(-0.5*sum(md2,2))
 
 for i = 1:size(M1,1)
   line( [ M1(i,1) , M2(i,1)] ...
	 ,[ M1(i,2) , M2(i,2)]);
  end

 
 %Compute particles close to "true" location
 disp('Finding "near by" particles.');
 dx = odo_last(:,1) - xyr1(1);
 dy = odo_last(:,2) - xyr1(2);
 %da = odo_last(:,3) - xyr1(3);
 
 DY_RANGE = 0.1;
 DX_RANGE = 0.1;
 %DA_RANGE = 5*pi/180;
 
 i1 = find(abs(dx) < DX_RANGE);
 i2 = find(abs(dy) < DY_RANGE);
 %i3 = find(abs(da) < DA_RANGE);
 
 i = intersect(i1,i2);
 %i = intersect(i,i3);
 
 disp(sprintf('Found %d "near by" particles.',length(i)));
 plot_robot(odo_last(i,:),'r.-',0.2);

 %Plot "true pose" 
 h = plot_robot(xyr1,'bo-',0.3);
 set(h,'LineWidth',3);
 orient landscape