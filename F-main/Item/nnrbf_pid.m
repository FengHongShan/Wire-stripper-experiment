function [sys, x0, str, ts] = nnrbf_pid( t,x,u,flag,T,nn,K_pid,...
    eta_pid, xite, alfa, beta0, w0)
switch flag,
case 0, [sys, x0, str, ts] = mdlInitializeSizes( T,nn) ;
case 2, sys = mdlUpdates( u) ;
case 3, sys = mdlOutputs( t, x, u, T, nn, K_pid,eta_pid,...
        xite, alfa, beta0, w0) ;
case { 1, 4, 9}, sys = [] ;
otherwise, error ( ['Unhandled flag = ' , num2str(flag)]);
end

function [sys,x0,str,ts] = mdlInitializeSizes(T, nn)
sizes = simsizes; 
sizes. NumContStates = 0; 
sizes.NumDiscStates = 3;
sizes. NumOutputs = 4+ 5* nn; 
sizes.NumInputs = 9+15* nn;
sizes. DirFeedthrough = 1; 
sizes. NumSampleTimes =1;
sys= simsizes( sizes) ; 
x0= zeros( 3, 1) ; str= [ ] ; ts=[T 0] ;

function sys = mdlUpdates( u)
sys= [ u(1) - u(2) ; u(1) ; u(1) + u(3) - 2* u(2) ];

function sys = mdlOutputs( t, x, u,T, nn, K_pid, eta_pid,...
    xite, alfa, beta0, w0)
ci3= reshape( u(7: 6+ 3* nn) , 3, nn) ; ci2= reshape(u( 7+ 5* nn: 6+ 8* nn) , 3, nn) ;
ci1= reshape( u( 7+ 10* nn: 6+ 13* nn) , 3, nn) ;
bi3= u( 7+ 3* nn: 6+ 4* nn) ; bi2= u( 7+ 8*nn: 6+ 9* nn) ;
bi1= u( 7+ 13* nn: 6+ 14* nn) ; w3= u( 7+ 4* nn: 6+ 5* nn) ;
w2= u( 7+ 9* nn: 6+ 10* nn) ; w1= u( 7+ 14* nn: 6+ 15* nn) ; xx= u( [ 6; 4; 5] ) ;
if t== 0
ci1= w0( 1) * ones( 3, nn) ; bi1= w0( 2) *ones( nn, 1) ;
w1= w0( 3) * ones( nn, 1) ; K_pid0= K_pid;
else, K_pid0= u( end-2: end) ; 
end
for j= 1: nn
h(j, 1) = exp( - norm( xx- ci1( : , j) )^2/(2* bi1(j) * bi1(j))) ;
end
dym= u(4) - w1'* h; 
w= w1+ xite* dym* h+alfa* (w1- w2) + beta0*( w2- w3) ;
for j= 1: nn
dbi(j,1) = xite* dym* w1(j) * h(j) * (bi1(j) ^(- 3)) * norm( xx- ci1(:,j))^2;
dci( : ,j) = xite*dym* w1(j)* h(j) * (xx- ci1(:,j)) * (bi1(j)^(- 2) );
end
bi= bi1+ dbi+ alfa* (bi1- bi2) + beta0*(bi2- bi3) ;
ci= ci1+ dci+ alfa* (ci1- ci2) + beta0*(ci2- ci3) ;
dJac= sum( w.*h.*( - xx (1) + ci (1,:)' ) ./bi.^2) ; % Jacobian
KK(1)= K_pid0(1)+ u(1) * dJac* eta_pid(1)* x(1);
KK(2)= K_pid0(2)+ u(1) * dJac* eta_pid(2)* x(2); 
KK(3)= K_pid0(3)+ u(1) * dJac* eta_pid(3)* x(3); 
sys= [ u( 6) + KK* x; KK'; ci( : ) ; bi( : ) ; w( : ) ] ;