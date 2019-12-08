function [timeElapsed,cost1,C,D,Optimal_path] =Astar_reeds(obs,xs,ys,xt,yt,xm,v,vw)

%DEFINE THE 2-D MAP ARRAY
tic
MAX_X=xm;
MAX_Y=xm;
r=(500/xm);

%This array stores the coordinates of the map and the 
%Objects in each coordinate
MAP=2*(ones(MAX_X,MAX_Y));

xTarget=xt;%X Coordinate of the Target
yTarget=yt;%Y Coordinate of the Target

MAP(xt,yt)=0;%Initialize MAP with location of the target

xStart=xs;%Starting Position
yStart=ys;%Starting Position

MAP(xs,ys)=1;

a=obs;
l=length(obs);
%a = [1 5; 1 7; 2 10; 2 8; 3 3; 3 4; 3 7; 4 2; 4 4;  5 2; 5 6; 5 7; 6 4; 6 5; 6 7; 7 1; 7 6; 7 8; 8 8; 8 5; 9 3; 9 9; 10 7; 10 5];
for h=1:l
    xval= a(h,1);
    yval= a(h,2);
    MAP(xval,yval)=-1;%Put on the closed list as well
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%LISTS USED FOR ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%OPEN LIST STRUCTURE
%--------------------------------------------------------------------------
%IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
%--------------------------------------------------------------------------
OPEN=[];
%CLOSED LIST STRUCTURE
%--------------
%X val | Y val |
%--------------
% CLOSED=zeros(MAX_VAL,2);
CLOSED=[];

%Put all obstacles on the Closed list
k=1;%Dummy counter
for i=1:MAX_X
    for j=1:MAX_Y
        if(MAP(i,j) == -1)
            CLOSED(k,1)=i; 
            CLOSED(k,2)=j; 
            k=k+1;
        end
    end
end
CLOSED_COUNT=size(CLOSED,1);
%set the starting node as the first node
xNode=xs;
yNode=ys;
A=2;
theta1=1.57;
C=0;
D=0;

OPEN_COUNT=1;
path_cost=0;
reedsConnObj = robotics.ReedsSheppConnection;
[~,goal_distance] = connect(reedsConnObj,[(xNode-0.5)*r (yNode-0.5)*r theta1],[(xTarget-0.5)*r (yTarget-0.5)*r 0]);
%goal_distance= distance_reed((xNode-0.5)*r,(yNode-0.5)*r,(xTarget-0.5)*r,(yTarget-0.5)*r,theta1,0);
OPEN(OPEN_COUNT,:)=insert_open_reed(xNode,yNode,xNode,yNode,path_cost,goal_distance,goal_distance,A,theta1);
OPEN(OPEN_COUNT,1)=0;
CLOSED_COUNT=CLOSED_COUNT+1;
CLOSED(CLOSED_COUNT,1)=xNode;
CLOSED(CLOSED_COUNT,2)=yNode;
NoPath=1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)
% plot(xNode+.5,yNode+.5,'go');
 %exp_array=expand_array(xNode,yNode,path_cost,xTarget,yTarget,CLOSED,MAX_X,MAX_Y);
 
 exp_array=expand_array_reed(xNode,yNode,path_cost,xTarget,yTarget,A,theta1,obs,r,v,CLOSED,MAX_X,MAX_Y);
 exp_count=size(exp_array,1);
 %UPDATE LIST OPEN WITH THE SUCCESSOR NODES
 %OPEN LIST FORMAT
 %--------------------------------------------------------------------------
 %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
 %--------------------------------------------------------------------------
 %EXPANDED ARRAY FORMAT
 %--------------------------------
 %|X val |Y val ||h(n) |g(n)|f(n)|
 %--------------------------------
 for i=1:exp_count
    flag=0;
    for j=1:OPEN_COUNT
        if(exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3) )
            OPEN(j,8)=min(OPEN(j,8),exp_array(i,5)); %#ok<*SAGROW>
            if OPEN(j,8)== exp_array(i,5)
                %UPDATE PARENTS,gn,hn
                OPEN(j,4)=xNode;
                OPEN(j,5)=yNode;
                OPEN(j,6)=exp_array(i,3);
                OPEN(j,7)=exp_array(i,4);
                OPEN(j,9)=exp_array(i,6);
                OPEN(j,10)=exp_array(i,7);

            end%End of minimum fn check
            flag=1;
        end%End of node check
%         if flag == 1
%             break;
    end%End of j for
    if flag == 0
        OPEN_COUNT = OPEN_COUNT+1;
        OPEN(OPEN_COUNT,:)=insert_open_reed(exp_array(i,1),exp_array(i,2),xNode,yNode,exp_array(i,3),exp_array(i,4),exp_array(i,5),exp_array(i,6),exp_array(i,7));
    end%End of insert new element into the OPEN list
 end%End of i for
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %END OF WHILE LOOP
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %Find out the node with the smallest fn 
 index_min_node = min_fn_reed(OPEN,OPEN_COUNT,xTarget,yTarget);
 %index_min_node 
 if (index_min_node ~= -1)    
   %Set xNode and yNode to the node with minimum fn
  xNode=OPEN(index_min_node,2);
  yNode=OPEN(index_min_node,3);
  A=OPEN(index_min_node,9);
  theta1=OPEN(index_min_node,10);
  path_cost=OPEN(index_min_node,6);%Update the cost of reaching the parent node
  if(isinf(path_cost))
      NoPath = 0;
      cost1=0;
      Optimal_path=0;
      timeElapsed =0;
      return
  end    
  %Move the Node to list CLOSED
  CLOSED_COUNT=CLOSED_COUNT+1;
  CLOSED(CLOSED_COUNT,1)=xNode;
  CLOSED(CLOSED_COUNT,2)=yNode;
  OPEN(index_min_node,1)=0;
 else
     NoPath=0;%Exits the loop!
 end%End of index_min_node check
end%End of While Loop

    

%Once algorithm has run The optimal path is generated by starting of at the
%last node(if it is the target node) and then identifying its parent node
%until it reaches the start node.This is the optimal path

i=size(CLOSED,1);
Optimal_path=[];
xval=CLOSED(i,1);
yval=CLOSED(i,2);
%xval=xNode;
%yval=yNode;
i=1;
Optimal_path(i,1)=xval;
Optimal_path(i,2)=yval;
i=i+1;
C=[];
D=[];
cost1=0;
B=[];
if ((xval == xTarget) && (yval == yTarget))
   %inode=0
   %Traverse OPEN and determine the parent nodes
   parent_x=OPEN(node_index(OPEN,xval,yval),4);%node_index returns the index of the node
   parent_y=OPEN(node_index(OPEN,xval,yval),5);
   
   while( parent_x ~= xStart || parent_y ~= yStart)
           Optimal_path(i,1) = parent_x;
           Optimal_path(i,2) = parent_y;
           %Get the grandparents:-)
           inode=node_index(OPEN,parent_x,parent_y);
           parent_x=OPEN(inode,4);%node_index returns the index of the node
           parent_y=OPEN(inode,5);
           i=i+1;
   end
 j=size(Optimal_path,1);
 j=j+1;
 Optimal_path(j,1) =xStart;
 Optimal_path(j,2) =yStart;
 %if(q==1)
 B = flipud(Optimal_path);
 %else
  % B = Optimal_path;  
 %end  
 b=1.57;
 u=2;
 
 
 for n= 1:(length(B)-1)
     x1 = B(n,1);
     y1 = B(n,2);
     x2 = B(n+1,1);
     y2 = B(n+1,2);
     if(x2>x1)
         k=1;
     elseif (x2 == x1)
         k=0;
     else
         k=-1;
     end  
     if(y2>y1)
         j=1;
     elseif (y2 == y1)
         j=0;
     else
         j=-1;
     end  
     [X,Y,t1,~] =cost((x1-0.5)*r,(y1-0.5)*r,b,(x2-0.5)*r,(y2-0.5)*r,v,0.1,v*vw,1.57,r,u,j,k,obs);
     C = [C X];
     D = [D Y];
     
     b=t1;
     if (j*k ~= 0)
       if(u==2 || u==-2)
           u=(j*k*u)/2;
       else
           u=2*j*k*u;
       end    
     end
 end
 d=0;    
 d = hypot(diff(C), diff(D));                            
 cost1 = sum(d);
 %plot(C,D,'Color', 'b', 'LineWidth', 2)
 timeElapsed = toc;
 %t=curve_reed(Optimal_path);
 
else
 C=0;
 D=0;
 B=0;
 %Optimal_path=0;
 timeElapsed = toc;
 cost1=0;
 %h=msgbox('Sorry, No path exists to the Target!','warn');
 %uiwait(h,5);
end

    





