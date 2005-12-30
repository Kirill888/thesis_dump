function F = find_fundamental_matrix(X1,X2)
	%Normalise X1 and X2,
	T1_norm = normalise_points(X1);
	X1_norm = T1_norm*X1;
	T2_norm = normalise_points(X2);
	X2_norm = T2_norm*X2;
	
	figure;
	plot(X1_norm(1,:),X1_norm(2,:),'r.', ...
	     X2_norm(1,:),X2_norm(2,:),'b.');
	
	u1 = X1_norm(1,:)';
	v1 = X1_norm(2,:)';
	u2 = X2_norm(1,:)';
	v2 = X2_norm(2,:)';
	
	A = [u2.*u1, u2.*v1, u2, v2.*u1, v2.*v1, v2, u1, v1, ones(size(u1))];
	[U,S,V] = svd(A);
	
	f = V(:,9)';
	
	F_norm = [f(1:3) ; f(4:6); f(7:9)];
	
	[U,S,V] = svd(F_norm);
	
	S(3,3) = 0;
	F_norm = U*S*(V');
	
	F = (T2_norm')*F_norm*T1_norm;
	
	return
