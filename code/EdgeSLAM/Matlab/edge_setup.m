function edge_setup(do_plot)

global LEFT_MASK
global RIGHT_MASK
global C1 C2 A1 A2
global DIST_MAPPING
global SEARCH_BOUND
global X_MAPPING
global Y_MAPPING
global SIGMA2_A1
global SIGMA2_A2
global H_OFFSET
global N_ROWS
global N_COLS

global F1
global F2
global CX1
global CX2

global cc_left fc_left alpha_left KK_left kc_left
global cc_right fc_right alpha_right KK_right kc_right



LEFT_MASK = 'undist_left_%05d.ppm';
RIGHT_MASK = 'undist_right_%05d.ppm';

MAX_DIST = 30000;
MIN_DIST = 1000;
OFFSET = 10;
H_OFFSET=+5;

N_ROWS = 240;
N_COLS = 640;

%T = [-295.8511; -5.8821; -48.1795];

%Calculated from Essential matrix
T = [ -1; 0.0036; -0.0119];
%Assume baseline is 295mm
T = T*295;

C1 = [0,0,0];
%C2 = [T(3),T(1),-0.0394];
C2 = [T(3),T(1),-0.05*pi/180];

%Calibration 25-May-2005 Field 1 only
%-- Focal length:
fc_left  = [ 743.470242388778843 ; 381.712232342261245 ];
fc_right = [ 742.533836317635974 ; 381.227925436133660 ];

%-- Principal point:
cc_left  = [ 321.739754578205122 ; 133.328348954646998 ];
cc_right = [ 316.793161616604436 ; 139.894390847480281 ];

%-- Distortion coefficients:
kc_left  = [ -0.277879908713581 ; 0.338025278957328 ; 0.001235213422044 ; 0.000461397587774 ; 0.000000000000000 ];
kc_right = [ -0.279192371042475 ; 0.341989952515136 ; 0.001202535267284 ; 0.000680246531665 ; 0.000000000000000 ];

%if 1
%fc_left = [737.5912; 757.7743];
%cc_left = [314.2927; 312.8159];
%kc_left = [-0.4423 ; 1.7283 ; 0.0007 ; 0.0071 ; 0.0000];

%fc_right = [711.0105; 737.6519];
%cc_right = [333.1539; 306.8209];
%kc_right = [-0.6231 ; 1.6220 ; -0.0051 ; -0.0343 ; 0.0000];
%else

%fc_left = [ 746.582651292947958 ; 765.684361616883848 ];
%cc_left = [ 315.407604460772689 ; 275.757612137191018 ];
%kc_left = [ -0.279711937054711 ; 0.353985966157746 ; 0.001225569584654 ; 0.000352184969869 ; 0.000000000000000 ];

%fc_right = [ 743.595094904625626 ; 762.197889126493806 ];
%cc_right = [ 327.790793471670497 ; 264.684682196933124 ];
%kc_right = [ -0.264188632265466 ; 0.167006293332836 ; -0.002462954030899 ; -0.000929003414421 ; 0.000000000000000 ];
%end

KK_left = [fc_left(1)     0       cc_left(1);
	       0      fc_left(2)  cc_left(2) ; 0 0 1];

KK_right = [fc_right(1)      0      cc_right(1);
	        0       fc_right(2) cc_right(2) ; 0 0 1];

F1  = fc_left(1);
F2  = fc_right(1);
CX1 = cc_left(1);
CX2 = cc_right(1);

im_columns = 1:640;

A1 = atan2(cc_left(1) - im_columns, ...
	   ones(size(im_columns))*fc_left(1));

A2 = atan2(cc_right(1) - im_columns, ...
	   ones(size(im_columns))*fc_right(1))+C2(3);

PIXEL_ERROR2 = 3*3;

f = fc_left(1);
x = cc_left(1) - im_columns;

SIGMA2_A1 = PIXEL_ERROR2.*f.*f./(f*f + x.*x)./(f*f + x.*x);
f = fc_right(1);
x = cc_right(1) - im_columns;
SIGMA2_A2 = PIXEL_ERROR2.*f.*f./(f*f + x.*x)./(f*f + x.*x);



dx = C2(1) - C1(1);
dy = C2(2) - C1(2);

L1 = zeros(640,640);
CA1 = cos(A1);
SA1 = sin(A1);

CA2  = cos(A2);
SA2  = sin(A2);
AUX1 = dx.*SA2 - dy.*CA2;

for i = 1:640
  AUX2 = CA1(i).*SA2 - SA1(i).*CA2;
  L1(i,:) = AUX1./AUX2;
end

ii = find(L1 < MIN_DIST | L1 > MAX_DIST);
L1(ii) = 0;
DIST_MAPPING = L1;

SEARCH_BOUND = zeros(640,2);

for i = 1:640
  ii = find(DIST_MAPPING(i,:) > 0);
  if isempty(ii)
    SEARCH_BOUND(i,1) = 1;
    SEARCH_BOUND(i,2) = 1;
  else
    SEARCH_BOUND(i,1) = ii(1);
    SEARCH_BOUND(i,2) = ii(end);
  end
end

ii = find(SEARCH_BOUND(:,1) < OFFSET);
SEARCH_BOUND(ii,1) = OFFSET*ones(length(ii),1);

ii = find(SEARCH_BOUND(:,2) > 640 - OFFSET);
SEARCH_BOUND(ii,2) = (640-OFFSET)*ones(length(ii),1);

for i = 1:640
  X_MAPPING(i,:) = DIST_MAPPING(i,:).*CA1(i) + C1(1);
  Y_MAPPING(i,:) = DIST_MAPPING(i,:).*SA1(i) + C1(2);
end



if nargin < 1
  do_plot = 0;
end

if do_plot
  figure;
  subplot(2,1,1);
  plot(C1(1),C1(2),'ro',C2(1),C2(2),'bs'); 
  hold on; axis equal;

  L = MAX_DIST;
  lx1 = L*cos(A1(1:10:end));
  ly1 = L*sin(A1(1:10:end));
  lx2 = L*cos(A2(1:10:end));
  ly2 = L*sin(A2(1:10:end));

  n1 = ones(size(lx1));

  line([lx1;n1*C1(1)],[ly1;n1*C1(2)],'Color','r');
  line([lx2;n1*C2(1)],[ly2;n1*C2(2)],'Color','b');

  subplot(2,1,2);

  x = [];
  y = [];

  for i = 1:640
    l = L1(i,:);
    l = l(find(l>0));
    x = [x,l*CA1(i) + C1(1)];
    y = [y,l*SA1(i) + C1(2)];
  end

  plot(x,y,'b.','MarkerSize',1);axis equal;
end
