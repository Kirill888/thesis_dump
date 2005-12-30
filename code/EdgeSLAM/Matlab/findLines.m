function l2 = findLines(l1)
% function l2 = findLines(l1)
%
% Find match for lines from I1 in I2
%  l1 -- lines in image I1
%  I1,I2 -- global variables - left and right images
%  SEARCH_BOUND -- global variable maps a column in left image
%                  to column range in the right image
%
%  H_OFFSET -- global variable, difference in pixels between image
%              1 and 2
%

global I1
global I2
global SEARCH_BOUND
global H_OFFSET

W = -3:3;

%[l1(2),l1(3)]+H_OFFSET

T  = I1(l1(2):l1(3),l1(1)+W,:);
I2_ = I2([l1(2):l1(3)]+H_OFFSET,SEARCH_BOUND(l1(1),1):SEARCH_BOUND(l1(1),2),:);


CORR_THRESHOLD = 0.6;
x = mycorr2(T,I2_);

l2 = findPeaks(x,CORR_THRESHOLD);
l2 = [l2+SEARCH_BOUND(l1(1),1)-1, x(l2)];


return

figure;
subplot(2,2,1); 
imshow(I1(:,:,:));
line([l1(1),l1(1)],[l1(2),l1(3)],'LineWidth',2);
subplot(2,2,2); 
imshow(I2(:,:,:));
hold on;
line([l2(:,1)';l2(:,1)'],[ones(1,size(l2,1))*l1(2);...
		ones(1,size(l2,1))*l1(3)],'LineWidth',2);

subplot(2,1,2); 
offset = SEARCH_BOUND(l1(1),1);

plot(offset:(offset+length(x)-1),x);hold on;
plot(l2(:,1),l2(:,2),'ro',...
     [1,(offset+length(x)-1)],[CORR_THRESHOLD,CORR_THRESHOLD],'r-');