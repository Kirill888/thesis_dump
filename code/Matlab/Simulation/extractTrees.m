function [obs] = extractTrees(las)
% function [obs] = extractTrees(las)
%
%  obs = [T nObs Range bearing radius]
%
%

  n = size(las,1);
  global AAr; 
  AAr = linspace(0,pi,361);
  
  obs = zeros(1000,5);
  nobs_all = 0;
  
  hw = waitbar(0, 'Extracting Trees');

  for i =1:n
    T = las(i,1);
    xra = detectTrees(las(i,2:362))';
    
    nobs = size(xra,1);
    
    if nobs > 0
      ind = (nobs_all+1):(nobs_all + nobs);
      obs(ind,:) = [ones(nobs,1)*T, ones(nobs,1)*nobs, xra];
      
      nobs_all = nobs_all + nobs;
    end
    
    if mod(i,100) == 0
      waitbar(i/n,hw);
    end
    
  end
  
  close(hw);
  obs = obs(1:nobs_all,:);
  obs(:,4) = obs(:,4) - pi/2;
  obs(:,5) = obs(:,5)*0.5;

