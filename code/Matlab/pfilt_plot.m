function pfilt_plot(base)

  if nargin < 1
    base = 'dat';
  end

   global obs_all pw podo pind


   if isempty(pw) | nargin > 0
      disp('Loading weights');
      dat  = load([base '_pfilt.pw']);
      pw   = dat(1:2:size(dat,1),:);
      pind = dat(2:2:size(dat,1),:);
   end

   if isempty(podo) | nargin > 0
      disp('Loading particle odometry log');
      podo = load([base '_pfilt.odo']);
   end


   %Display things
   ESC = 27;


   numF = size(podo,1);
   i = 1;
   c = 0;

   fh = gcf;
   h = do_display(i,[],obs_all,pw(i,:),podo(i,:));
   axis equal
   grid on
   hold on

   keep = 0;
   keepON = 0;

   while 1

      while waitforbuttonpress == 0
      end

      c = get(gcf,'CurrentCharacter');

      step = 0;

      switch(c)
        case {'n',' '}
          step = +1;
        case {'N'}
          step = +10;
        case 'p'
          step = -1;
        case 'P'
          step = -10;

	case {'k'}
	  keep = 1;

	case {'K'}
	  keepON = ~keepON;

        case {'q',ESC}
           return;
        otherwise
      end


      if step ~= 0
        i = i + step;
        i = max(1   , i);
        i = min(numF, i);

        figure(fh)
	if ~keep & ~keepON
	  delete(h);
	end

%	disp(i)
%	size(odo)
%	size(podo)
%	size(pw)

        h = do_display(i, [],obs_all,pw(i,:),podo(i,:));
	keep = 0;
      end

   end

 
function h_all = do_display(k,odo,obs_all,pw,podo)
  n = size(pw,2);
  NBEST = floor(n/30);

  %ind = find(obs_all(:,7) == k);
  %h1 = mplot(obs_all(ind,:),'gs','g:');
  
  h1 = [];

  if ~isempty(odo)
    h2 = plot_robot(odo , 'go-', 0.05);
  else
    h2 = [];
  end

  hold on

  np = size(podo,2)/3;

  o = zeros(3,np);
  o(:) = podo;
  o    = o';
%  o(:,3) = o(:,3)-pi/2;
  
  h3 = plot_robot(o, 'ro-', 0.05);

  %Plot NBEST particles
  [v,ind] = sort(pw);


  ind1 = ind(n:-1:max(1,n-NBEST));
  ind2 = ind(1:+1:min(n,NBEST));

  h4 = plot(o(ind1,1),o(ind1,2),'bs', ...
            o(ind2,1),o(ind2,2),'g<');

  h_all = [h1(:); h2(:); h3(:); h4(:)];
