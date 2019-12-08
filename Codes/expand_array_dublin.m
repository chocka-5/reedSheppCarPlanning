function exp_array = expand_array_dublin(node_x,node_y,hn,xTarget,yTarget,A,theta1,r2,ra,v,CLOSED,MAX_X,MAX_Y)
    %Function to return an expanded array
    %This function takes a node and returns the expanded list
    %of successors,with the calculated fn values.
    %The criteria being none of the successors are on the CLOSED list.
    %
    %   Copyright 2009-2010 The MathWorks, Inc.
    reedsConnObj = robotics.DubinsConnection;
    exp_array=[];
    %path={};
    exp_count=1;
    c2=size(CLOSED,1);%Number of elements in CLOSED including the zeros
    if (A==2 || A==-2)
        for r =1:length(r2)
            if((r2(r,1)==node_x)&& (r2(r,2)== node_y+1))
                a=[];
                break;
            else
                a=A/2;
            end
        end
        for k= 1:-1:-1
            for j=a
              %if (k~=1 || k~=0)  %The node itself is not its successor
                s_x = node_x+k;
                s_y = node_y+j;
                if( (s_x >0 && s_x <=MAX_X) && (s_y >0 && s_y <=MAX_Y))%node within array bound
                    flag=1;                    
                    for c1=1:c2
                        if(s_x == CLOSED(c1,1) && s_y == CLOSED(c1,2))
                            flag=0;
                        end
                    end %End of for loop to check if a successor is on closed list.
                    if (flag == 1)
                       exp_array(exp_count,1) = s_x;
                       exp_array(exp_count,2) = s_y;
                       if (k ==0)
                           angle=A;
                %           c=1*ra;
                       else
                           angle=(k*A*j)/2;
                 %          c=1.414*ra;
                       end
                       [~,~,theta,c] = cost((node_x-0.5)*ra,(node_y-0.5)*ra,theta1,(s_x-0.5)*ra,(s_y-0.5)*ra,v,0.1,0,0,ra,A,j,k,r2);
                       theta=round(theta,4);
                       %{
                       t=theta-0.785;
                       if (tan(t)>0)
                           if (sin(t)>0)
                               angle=2;
                           else
                               angle=-2;
                           end
                       else
                           if (sin(t)>0)
                               angle=-1;
                           else
                               angle=1;
                           end 
                       end    
                       %}
                       %{
                        if (angle == -2)
                            th=-1.57;
                        end  
                        if (angle == 2)
                            th=1.57;
                        end  
                        if (angle ==1)
                            th=0;
                        end  
                        if (angle == -1)
                            th=3.14;
                        end
                        %}
                       %c=round(0.7*c+0.51,1);
                       %if (c>1 && c<2)
                        %    c=1.414;
                       %end
                      
                       exp_array(exp_count,6) = angle;
                       exp_array(exp_count,7) = theta;
                       exp_array(exp_count,3) = hn+c;%sqrt((node_x -s_x)^2 + (node_y-s_x)^2);
                      
                        %exp_array(exp_count,3) = hn+distance_reed(node_x,node_y,s_x,s_y,A,angle);
                       if((s_x == xTarget) && (s_y == yTarget))
                           exp_array(exp_count,4) = 0;
                         
                       else
                           %exp_array(exp_count,4) =abs(distance((xTarget-0.5)*ra,(yTarget-0.5)*ra,(s_x-0.5)*ra,(s_y-0.5)*ra,0,theta));
                           [~,e2] = connect(reedsConnObj,[(xTarget-0.5)*ra (yTarget-0.5)*ra 0],[(s_x-0.5)*ra (s_y-0.5)*ra theta]);
                           exp_array(exp_count,4) =abs(e2);
                       end
                       
                       exp_array(exp_count,5)= exp_array(exp_count,3)+exp_array(exp_count,4); 
                       %pause(20)
                       exp_count=exp_count+1;
                    end%Populate the exp_array list!!!
                end% End of node within array bound
              %end%End of if node is not its own successor loop
            end%End of k for loop    
        end
    end   
   if (A == 1 || A == -1)
       for r =1:length(r2)
            if((r2(r,1)==node_x+1)&& (r2(r,2)== node_y))
                b=[];
                break;
            else
                b=A;
            end
       end
       for k= b
         for j= 1:-1:-1
            s_x = node_x+k;
            s_y = node_y+j;
            if( (s_x >0 && s_x <=MAX_X) && (s_y >0 && s_y <=MAX_Y))%node within array bound
                flag=1;                    
                for c1=1:c2
                    if(s_x == CLOSED(c1,1) && s_y == CLOSED(c1,2))
                        flag=0;
                    end
                 end %End of for loop to check if a successor is on closed list.                    
                if (flag == 1)
                    exp_array(exp_count,1) = s_x;
                    exp_array(exp_count,2) = s_y;

                    if (j == 0)
                        exp_array(exp_count,6) = A;
                        angle=A;
                        %c=1*ra;

                    else
                        exp_array(exp_count,6) = 2*k*A*j;
                        angle=2*k*A*j;
                        %c=1.414*ra;

                    end
                    %{
                    if (angle == -2)
                        th=-1.57;
                    end  
                    if (angle == 2)
                        th=1.57;
                    end  
                    if (angle ==1)
                        th=0;
                    end  
                    if (angle == -1)
                        th=3.14;
                    end
                    %}
                   
                   %{ 
                    t=theta-0.785;
                    if (tan(t)>0)
                       if (sin(t)>0)
                           angle=2;
                       else
                           angle=-2;
                       end
                    else
                       if (sin(t)>0)
                           angle=-1;
                       else
                           angle=1;
                       end 
                    end
                     %}
                    %c=round(0.7*c+0.51,1);
                    %if (c>1 && c<2)
                     %   c=1.414;
                    %end    
                    [~,~,theta,c] = cost((node_x-0.5)*ra,(node_y-0.5)*ra,theta1,(s_x-0.5)*ra,(s_y-0.5)*ra,v,0.1,0,0,ra,A,j,k,r2);
                    theta=round(theta,4);
                    exp_array(exp_count,7) = theta;
                    exp_array(exp_count,6) = angle;
                    exp_array(exp_count,3) = hn+c;%sqrt((node_x -s_x)^2 + (node_y-s_x)^2);
                    %e1 = hn+c;%sqrt((node_x -s_x)^2 + (node_y-s_x)^2);
                    %exp_array(exp_count,3) = hn+c;
                    %e1=hn+c;%exp_array(exp_count,3);%cost of travelling to node
                    %exp_array(exp_count,3) = hn+distance_reed(node_x,node_y,s_x,s_y,A,angle);
                    if((s_x == xTarget) && (s_y == yTarget))
                       exp_array(exp_count,4) = 0;
                      
                    else 
                      % exp_array(exp_count,4) =abs(distance((xTarget-0.5)*ra,(yTarget-0.5)*ra,(s_x-0.5)*ra,(s_y-0.5)*ra,0,theta));
                       %e2=abs(distance((xTarget-0.5)*ra,(yTarget-0.5)*ra,(s_x-0.5)*ra,(s_y-0.5)*ra,0,theta));
                       [~,e2] = connect(reedsConnObj,[(xTarget-0.5)*ra (yTarget-0.5)*ra 0],[(s_x-0.5)*ra (s_y-0.5)*ra theta]);
                       exp_array(exp_count,4) =abs(e2);
                    end
                    exp_array(exp_count,5)= exp_array(exp_count,3)+exp_array(exp_count,4); 
                    %exp_array(exp_count,5) =e1+e2;
                    %pause(20)
                    exp_count=exp_count+1;
                end%Populate the exp_array list!!!
            end% End of node within array bound
       % end%End of if node is not its own successor loop
         end%End of k for loop    
       end
   end   
                        
                       