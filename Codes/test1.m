na=0;                    %na  , nd is used to count the no of non path generating cases
nd=0;
for i=1%:15
    if (i==1)
        gr=3;
        v=5;                 %gr -- grid size   v velocity of vehicle vw -wind velocity obsperc - obstacle percentage
        vw=0.5;
        obsperc=10;
    elseif (i==2)
        gr=5;
        v=5;
        vw=0.5;
        obsperc=10;
    elseif (i==3)
        gr=10;
        v=5;
        vw=0.5;
        obsperc=10;
    elseif (i==4)
        gr=15;
        v=5;
        vw=0.5;
        obsperc=10;
    elseif (i==5)
        gr=20;
        v=5;
        vw=0.5;
        obsperc=10;
    elseif (i==6)
        gr=5;
        v=8;
        vw=0.5;
        obsperc=10; 
    elseif (i==7)
        gr=5;
        v=10;
        vw=0.5;
        obsperc=10;
    elseif (i==8)
        gr=5;
        v=5;
        vw=0.5;
        obsperc=1;
    elseif (i==9)
        gr=5;
        v=5;
        vw=0.5;
        obsperc=5; 
    elseif(i==10)
        gr=5;
        v=5;
        vw=0.5;
        obsperc=15; 
    elseif(i==11)
        gr=5;
        v=5;
        vw=0.5;
        obsperc=20;   
    elseif(i==12)
        gr=5;
        v=5;
        vw=0.1;
        obsperc=10; 
    elseif(i==13)
        gr=5;
        v=5;
        vw=0.3;
        obsperc=10; 
     elseif(i==14)
        gr=5;
        v=5;
        vw=0.7;
        obsperc=10;
     elseif(i==15)
        gr=5;
        v=5;
        vw=0.9;
        obsperc=10;   
    end 
    %obsperc=10;
    %figure(i)
    %grid on
    %hold on
    noc=floor(500/gr);              
    nn = floor(0.3*noc);                      % start and end location need to be in either corner of grid box 
    %axis([0 gr*noc 0 gr*noc])                % for ex if grid size is 5,it will be 100 *100 grid, gr=5 , noc =100                                                                      
    %set(gca,'xtick',[0:gr:gr*noc])           %this implies start will be randomly assigned between 30*30 and end 
    %set(gca,'ytick',[0:gr:gr*noc])           %will be assigned between 70*70 - 100*100 , i.e final 30*30 grid box nn=30
    %s=randi(2);
    %s=1;
    %if (s==1)
    xs=randi(nn);                               %xs- start x coordinate , ys - start y coordinate
    ys=randi(nn);                               %xf - end coordinate loc yf end y coordinate
    xf=noc+1-randi(nn);
    yf=noc+1-randi(nn);
    %else
      %xs=noc+1-randi(3);   
      %ys=randi(3);  
      %xf=randi(3);
     % yf=noc+1-randi(3);
    %end
    %xs=8;
    %ys=22;
    %xf=164;
    %yf=138;
    noo=round((noc*noc*obsperc)/100);            %noo no of obstaclles 
    %rectangle('Position',[gr*(xf-1) gr*(yf-1) gr gr],'FaceColor',[0 1 0]);
      %text(gr*(xf),gr*(yf),'Target')
    %rectangle('Position',[gr*(xs-1) gr*(ys-1) gr gr],'FaceColor',[0 0 1]);
      %text(gr*(xs),gr*(ys),'Start')
    r2 = randi(noc,noo,2);                          %r2 has list of obstacles   
    for r =1:noo                                    %for loop is to ensure the start and end location arent alloted has obstacles  
           if((r2(r,1)== xf && r2(r,2)== yf) || (r2(r,1)== xs && r2(r,2)== ys))
               if (r== noo)
                   noo=noo-1;
               else
                   r2(r,1)=r2(r+1,1);
                   r2(r,2)=r2(r+1,2);
               end
           end
    end     

    for e =1:noo
           if((r2(e,1)== xs && r2(e,2)== ys) || (r2(e,1)== xf && r2(e,2)== yf)) 
               if (e== noo)
                   noo=noo-1;
               else
                   r2(e,1)=r2(e+1,1);
                   r2(e,2)=r2(e+1,2);
               end
           end
    end
 %{
    l=length(r2);
    for h=1:l
       xval= r2(h,1);
       yval= r2(h,2);
       rectangle('Position',[gr*(xval-1) gr*(yval-1) gr gr],'FaceColor',[1 0 0])
    end
  %}  
    [obs{i}] =r2;
    [sf{i}] =[xs,ys;xf,yf];
    
     [trd(i),costd(i),A,B,opd{i}]=A_Star1(r2,xs,ys,xf,yf,noc,v,vw);   %trd -time reqd for code to calculate the path 
     if (A ==0)                                                       %costd - cost of the optimal path                              
         costd(i)=0;                                                  %A - x coordinates of optimal path
         nd=nd+1;                                                     %B - y coordinates of optimal path
     else                                                             %OPd - x and y  coordinates of optimal path
          
          A=[A gr*(xf-0.5)];                                          %Opd and cost d are saved in cell for 100 variations
          B=[B gr*(yf-0.5)];
          %plot(gr*(G(:,1)-0.5),gr*(G(:,2)-0.5),'o','MarkerFaceColor',[0 0 0]);
          %plot(A,B,'Color', 'm', 'LineWidth', 2);
      
   
     end
     i
     [tr(i),costa(i),C,D,opr{i}]=Astar_reeds(r2,xs,ys,xf,yf,noc,v,vw);
     if (C ==0)
         costa(i)=0;
         na=na+1;
     else
          
          C=[C gr*(xf-0.5)];
          D=[D gr*(yf-0.5)];
          %plot(gr*(F(:,1)-0.5),gr*(F(:,2)-0.5),'o','MarkerFaceColor',[0 0 1]);
          %plot(C,D,'Color', 'r', 'LineWidth', 2);
     end
    
    %[trr(i),costr(i),E,B] =rrtreed(r2,xs,ys,xf,yf,noc);
     %plot(gr*E,gr*B,'Color', 'y', 'LineWidth', 2)
     
     %legend({'Dubin  A*','Reed Shepp A*','Control Reed Shepp A*','RRT*'},'Location','northwest','FontSize',7)
     
     %format = '%d.jpeg';
     %str = sprintf(format,i);
     %saveas(figure(i),str);
    % if (E ==0)
     %    nr=nr+1;
     %end
    
end

 
 