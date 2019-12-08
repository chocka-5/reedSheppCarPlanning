function [E,B,theta,c] = cost(x1,y1,theta1,x2,y2,v,dt,vw,thetaw,ra,A,j,k,r2)
c=0;
E=0;
B=0;

xc=x1;
yc=y1;
i=1;
if (A*j==-2)
  theta1 =-theta1;
end  
if (A*k==-1)
 theta1 =3.14-theta1;
end

while(abs(sqrt((xc-x2)^2 + (yc-y2)^2)) >= 0.6*ra)
    [x,y,theta] = NLGL1(xc,yc,x1,y1,x2,y2,theta1,v,dt,vw,thetaw);
    %plot(x,y,'o') 
    theta=ang_wrap(theta);
    xo=ceil(x/ra);
    yo=ceil(y/ra);
    flag=1;
   %{
    for r =1:length(r2)
        if((r2(r,1)== xo )&& (r2(r,2)== yo))
             c=Inf;
             flag=0;
        end
        dx=r2(r,1)*ra- x;
        dy=r2(r,2)*ra- y;
        if (((dx)^2+(dy)^2)<0.5*ra)
             c=Inf;
             flag=0;
        end
    end
   %}
    if(flag == 1)
        c=c+v*dt;    
    end    
    xc=x;
    yc=y;
    theta1=theta;
    E(i)=x;
    B(i)=y;
    i=i+1;
end 

%p=E(i-1);
%q=B(i-1);
if ( A*j== -2)
theta =theta-(3.14*j);
theta=ang_wrap(theta);
end
if (A*k == -1)
theta =theta-(3.14*k);
theta=ang_wrap(theta);
end

