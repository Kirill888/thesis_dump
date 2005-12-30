function las = las_filt(las0, filter_name, varargin)
% function las = las_filt(las0, filter_name, varargin)
%  Supported names:
%     median
%     mean
%     outlier
%     none
%


switch filter_name
 case 'median'
  las = filt_median(las0, varargin{1});
 case 'outlier'
  las = filt_outlier(las0, varargin{:});
 case 'mean'
  las = filt_mean(las0, varargin{1});
 case 'none'
  las = las0;
 
 otherwise
end

figure(1); clf;
a = linspace(0,pi,length(las));
ca = cos(a); sa = sin(a);
ii = find(las < inf);

xx0 = las0.*ca; yy0 = las0.*sa;
xx1 = las(ii).*ca(ii); yy1 = las(ii).*sa(ii);

plot(xx0,yy0,'g.-', xx1,yy1,'r.-'); axis equal
legend('original','filtered');



  

function las = filt_median(las0, windowSize)
  winsz2 = fix(windowSize/2);
  ind = -winsz2:(windowSize - winsz2);
  
  las = zeros(size(las0)) + inf;
  n = length(las0);
  
  for i = winsz2+1:n-(windowSize - winsz2)
    ii = ind + i;
    las(i) = median(las0(ii));
  end

function las = filt_outlier(las0, varargin)
  winsz2 = fix(varargin{1}/2);
  ind = -winsz2:(varargin{1} - winsz2);
  
  las = zeros(size(las0)) + inf;
  n = length(las0);
  
  for i = winsz2+1:n-(varargin{1} - winsz2)
    ii = ind + i;
    r  = median(las0(ii));
    
    err = r - las0(i);
    
    if abs(err) > varargin{2}
      las(i) = inf;
    else
      las(i) = las0(i);
    end
    
  end

  

function las = filt_mean(las0, windowSize)
  winsz2 = fix(windowSize/2);
  ind = -winsz2:(windowSize - winsz2);
  
  las = zeros(size(las0)) + inf;
  n = length(las0);
  
  for i = winsz2+1:n-winsz2
    ii = ind + i;
    las(i) = mean(las0(ii));
  end
