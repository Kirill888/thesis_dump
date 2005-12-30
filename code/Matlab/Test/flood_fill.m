function im = flood_fill(im0, mask, start_point)
% function im = flood_fill(im0, mask,start_point)
%
%  im0, mask -- binary images
%
%

sz_mask = size(mask);
if mod(sz_mask(1),2) == 0 || mod(sz_mask(2),2) == 0
  disp('Mask should has odd size');
  im = [];
  return;
end

global IM0 
global IM_looked
global IM
global MASK
global mr
global mc

mr = floor(sz_mask(1)/2)
mc = floor(sz_mask(2)/2)


IM0       = im0;
IM        = zeros(size(im0));
IM_looked = zeros(size(im0));

MASK = mask;

size(IM0)
int_flood_fill(start_point(1), start_point(2));


im = IM;

clear global IM0 IM mr mc MASK


function res = check_point(r,c)
global IM0 
global IM
global MASK
global mr
global mc

  c_min = c - mc;  c_max = c + mc;
  r_min = r - mr;  r_max = r + mr;
  
  res = 0; 
  
  if c_min < 1 || r_min < 1
    return;
  end
  
  if r_max > size(IM0,1) || c_max > size(IM0,2)
    return;
  end
  
  iic = c_min:c_max;
  iir = r_min:r_max;
  
  template0 = IM0(iir,iic);
  
  tmp = and(xor(template0, MASK), not(template0));

  if sum(tmp(:)) == 0  % If perfect match
    res = 1;
  end

function fillPoint(r,c)
global IM0 
global IM
global IM_looked
global MASK
global mr
global mc


  iic = (c-mc):(c+mc);  iir = (r-mr):(r+mr);
  IM(iir,iic) = or(IM(iir,iic), MASK);
  
  IM_looked(r,c) = 1;
  
%  disp(sprintf('Fill point %d,%d',r,c));

function int_flood_fill(r,c)
global IM0 
global IM
global IM_looked
global MASK
global mr
global mc

  %First check current point
  if ~check_point(r,c)
    return;
  end
  
  %Fill current point
  fillPoint(r,c);  

  %Go right
  i = 1;
  
  while check_point(r, c + i)
    fillPoint(r, c + i);
    i = i + 1;
  end
  
  max_r = c + i - 1;

  %Go left
  i = 1;
  
  while check_point(r, c - i)
    fillPoint(r, c - i);
    i = i + 1;
  end
  
  max_l = c - i + 1;
  
  %Check up and bottom
  for i = max_l:max_r
    
    %Check bottom
    if r > 1
      if ~IM_looked(r-1,i) && IM0(r-1,i)
	int_flood_fill(r-1,i);
      end
    end
    
    %Check top
    if r < size(IM0,1)
      if ~IM_looked(r+1,i) && IM0(r+1,i)
	int_flood_fill(r+1,i);
      end
    end
    
  end
  

  
    


