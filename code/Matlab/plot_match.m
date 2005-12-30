function plot_match(m1,m2,matches,m2in1)
%function plot_match(m1,m2,matches,m2in1)
%
%

  
  ind = find(matches > 0);

  if 1
    disp('Computing alignment');
    xyr = align_maps(m1.map(ind,:), m2.map(matches(ind),:),m2in1);
    
    
    disp(sprintf('New alignment: %.3f %.3f %.3f',xyr(1),xyr(2), ...
		 xyr(3)*180/pi));
    dd = m2in1 - xyr;
    
    disp(sprintf('Difference: %.3f %.3f %.3f\n',dd(1),dd(2),dd(3)* ...
		 180/pi));
  else
    xyr = m2in1;
  end
  
  if 1
    f1 = figure;
    plot_robot(m1.odo,'g.-',0.1); hold on; axis equal;
    h = plot(m1.las(:,1), m1.las(:,2),'b.'); set(h,'MarkerSize',1);
    h = mplot(m1.map,'gs','g-' ); set(h,'MarkerSize',12, ...
					'LineWidth',2);
    ax1 = axis;

    f2 = figure;
    plot_robot(m2.odo,'r.-',0.1); hold on; axis equal
    h = plot(m2.las(:,1), m2.las(:,2),'y.'); set(h,'MarkerSize',1);
    h = mplot(m2.map,'ro','r-'); set(h,'MarkerSize',8, ...
				       'LineWidth',2);
    ax2 = axis;
    
    ax = [0 0 0 0];
    ax([1,3]) = min(ax1([1,3]), ax2([1,3]));
    ax([2,4]) = max(ax1([2,4]), ax2([2,4]));

    axis(ax);
    figure(f1); axis(ax);
    
    figure
  end
  
  
  m2_ = map_translate(m2,xyr);
  
  %Extract landmarks with known correspondence
  M1  = m1.map(ind,:);
  M2  = m2_.map(matches(ind),:);
  
  

  %Extract 'free' landmarks
  ind = setdiff(1:size(m2.map,1),matches(ind));
  M2_ = m2_.map(ind,:);
  ind = find(matches < 1);
  M1_ = m1.map(ind,:);

  %Increase Ellipse
  scaler = 1;
  M1(:,3:6) =  scaler*M1(:,3:6);
  M2(:,3:6) =  scaler*M2(:,3:6);
  M1_(:,3:6) = scaler*M1_(:,3:6);
  M2_(:,3:6) = scaler*M2_(:,3:6);

  %Plot stuff
  
  %Robot path
  plot_robot(m1.odo,'g.-',0.1); hold on; axis equal;
  plot_robot(m2_.odo,'r.-',0.1);
  
  %Laser data
  h = plot(m1.las(:,1), m1.las(:,2),'b.'); set(h,'MarkerSize',1);
  h = plot(m2_.las(:,1), m2_.las(:,2),'y.'); set(h,'MarkerSize',1);
  
  %Overlapping features
  h = mplot(M1,'gs','g-' ); set(h,'MarkerSize',12,'LineWidth',2);
  h = mplot(M2,'ro','r-'); set(h,'MarkerSize', 8,'LineWidth',2);
  
% $$$   for i = 1:size(M1,1)
% $$$    h = line( [ M1(i,1) , M2(i,1)] ...
% $$$ 	 ,[ M1(i,2) , M2(i,2)]);
% $$$    set(h,'Color','b');
% $$$    set(h,'LineWidth',3);
% $$$   end

  %Free features
  h = mplot(M1_,'gs',''); set(h,'MarkerSize',8,'LineWidth',2);
  h = mplot(M1_,'gp',[]); set(h,'MarkerSize',25,'LineWidth',2);

  h = mplot(M2_,'ro',''); set(h,'MarkerSize',8,'LineWidth',2);
  h = mplot(M2_,'rp',[]); set(h,'MarkerSize',25,'LineWidth',2);
  