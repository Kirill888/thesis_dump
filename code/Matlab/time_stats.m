function time_stats(file)

  disp(sprintf('Loading: %s',file));
  run(file);
  
  t_odo      = timer_00;
  t_las      = timer_01;
  
  t_eval     = timer_02;
  t_resample = timer_03;
  t_check    = timer_04;
  
  t_match    = timer_05;
  t_loop     = timer_06;

if 0
  figure;
  
  plot( t_las(:,2)  , t_las(:,1),  'g-.' ...
       ,t_check(:,2), t_check(:,1),'go'  ... 
       ,t_match(:,2), t_match(:,1),'rx' ...
       ,t_loop(:,2) , t_loop(:,1), 'rs');
  
  legend('Laser','CheckPoint','Map Match', 'LoopClose');
  xlabel('Algorithm Running Time (seconds)');
  ylabel('Time to complete (seconds)');
end

  m_eval     = 1000*mean(t_eval(:,1));
  m_resample = 1000*mean(t_resample(:,1));
  m_odo      = 1000*mean(t_odo(:,1));
  ind = find(t_check(:,1) < 0.5);
  m_check    = 1000*mean(t_check(ind,1));
  
  ind = find(t_check(:,1) > 0.5);
  m_loop     = 1000*mean(t_check(ind,1));
  
  disp(sprintf(['Execution Times\n' ...
		' Eval      %6.2f ms\n' ...
		' Resample  %6.2f ms\n' ...
		' Odometry  %6.2f ms\n' ...
		' Check     %6.2f ms\n' ...
	        ' Loop      %6.2f ms\n'] ...
	       ,m_eval, m_resample, m_odo, m_check, m_loop));

