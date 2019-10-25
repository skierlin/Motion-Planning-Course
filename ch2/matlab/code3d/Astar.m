clear;
MAX_X=10;
MAX_Y=10;
MAX_Z=10;
MAX_VAL=10;  
%This array stores the coordinates of the map and the 
%Objects in each coordinate
MAP=2*(ones(MAX_X,MAX_Y,MAX_Z));
 % Obtain Obstacle, Target and Robot Position 
% Initialize the MAP with input values        
% Obstacle=-1,Target = 0,Robot=1,Space=2      
j=0;
x_val = 1;
y_val = 1;
Z_val = 1;
axis([1 MAX_X+1 1 MAX_Y+1 1 MAX_Z+1])  
grid on;                     
hold on;                     
n=0;%Number of Obstacles     

% BEGIN Interactive Obstacle, Target, Start Location selection

xval=10;           
yval=10;
zval=5;
xTarget=xval;%X Coordinate of the Target 
yTarget=yval;%Y Coordinate of the Target
zTarget=zval; 

MAP(xval,yval,zval)=0;%Initialize MAP with location of the target 
plot3(xval+.5,yval+.5,zval+.5,'gd');         
text(xval+1,yval+.5,zval+.5,'Target');       

xval=[2,3,4,5,6,4,2,3,5,6,8,7,5,6,7,8,9,3,4,4,4,3,4,4,6,8];
yval=[2,2,5,4,3,6,5,1,2,3,4,5,5,6,7,8,9,3,4,4,4,4,3,5,7,8];
zval=[2,4,3,5,2,1,2,4,8,6,4,5,5,6,7,8,9,1,2,3,4,1,1,2,3,4];
for an = 1:26
    MAP(xval(an),yval(an),zval(an))=-1;%Put on the closed list as well
    plot3(xval(an)+.5,yval(an)+.5,zval(an)+.5,'ro');   
end 
xval=1;
yval=1;
zval=1;
xStart=xval;
yStart=yval;
zStart=zval;
MAP(xval,yval,zval)=1;
plot3(xval+.5,yval+.5,zval+.5,'bo');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%OPEN LIST
%--------------------------------------------------------------------------
%IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
%--------------------------------------------------------------------------
OPEN=[];
%CLOSED LIST 
%--------------
%X val | Y val |
%--------------
% CLOSED=zeros(MAX_VAL,2);
CLOSED=[];

%Put all obstacles on the Closed list

k=1;%Dummy counter 

for i = 1:MAX_X
    for j = 1:MAX_Y
        for t = 1:MAX_Z
            if(MAP(i,j,t) == -1)
                CLOSED(k,1) = i; 
                CLOSED(k,2) = j;
                CLOSED(k,3) = t;
                k = k+1;
            end
        end
    end
end
CLOSED_COUNT=size(CLOSED,1);
%set the starting node as the first node
xNode = xval;
yNode = yval;
zNode = zval;
OPEN_COUNT = 1;
path_cost = 0;
goal_distance = distance(xNode,yNode,zNode,xTarget,yTarget,zTarget);
OPEN(OPEN_COUNT,:) = insert_open(xNode,yNode,zNode,xNode,yNode,zNode,path_cost,goal_distance,goal_distance); 
OPEN(OPEN_COUNT,1) = 0;
CLOSED_COUNT = CLOSED_COUNT+1;
CLOSED(CLOSED_COUNT,1) = xNode;
CLOSED(CLOSED_COUNT,2) = yNode;
CLOSED(CLOSED_COUNT,3) = zNode;
NoPath = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while((xNode ~= xTarget || yNode ~= yTarget || zNode ~= zTarget) && NoPath == 1)
%  plot(xNode+.5,yNode+.5,'go');
 exp_array=expand_array(xNode,yNode,zNode,path_cost,xTarget,yTarget,zTarget,CLOSED,MAX_X,MAX_Y,MAX_Z);   
 exp_count=size(exp_array,1);                                                       
 %UPDATE LIST OPEN WITH THE SUCCESSOR NODES
 %OPEN LIST FORMAT
 %EXPANDED ARRAY FORMAT
 for i=1:exp_count
    flag=0;
    for j=1:OPEN_COUNT
        if(exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3) && exp_array(i,3) == OPEN(j,4))
            OPEN(j,10)=min(OPEN(j,10),exp_array(i,6)); %#ok<*SAGROW>
            if OPEN(j,10)== exp_array(i,6)
                %UPDATE PARENTS,gn,hn
                OPEN(j,5)=xNode;
                OPEN(j,6)=yNode;
                OPEN(j,7)=zNode;
                OPEN(j,8)=exp_array(i,4);
                OPEN(j,9)=exp_array(i,5);
            end;%End of minimum fn check
            flag=1;
        end;%End of node check
%         if flag == 1
%             break;
    end;%End of j for
    if flag == 0
        OPEN_COUNT = OPEN_COUNT+1;
        OPEN(OPEN_COUNT,:)=insert_open(exp_array(i,1),exp_array(i,2),exp_array(i,3),xNode,yNode,zNode,exp_array(i,4),exp_array(i,5),exp_array(i,6));
     end;%End of insert new element into the OPEN list
 end;%End of i for
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
index_min_node = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget,zTarget);
  if (index_min_node ~= -1)    
   %Set xNode and yNode to the node with minimum fn  
   xNode=OPEN(index_min_node,2);
   yNode=OPEN(index_min_node,3);
   zNode=OPEN(index_min_node,4);
   path_cost=OPEN(index_min_node,8);%Update the cost of reaching the parent node
                          
  %Move the Node to list CLOSED
  CLOSED_COUNT = CLOSED_COUNT + 1;
  CLOSED(CLOSED_COUNT,1) = xNode;
  CLOSED(CLOSED_COUNT,2) = yNode;
  CLOSED(CLOSED_COUNT,3) = zNode;
  OPEN(index_min_node,1) = 0;
  else
      %No path exists to the Target!!
      NoPath = 0;%Exits the loop!
  end;%End of index_min_node check
end;%End of While Loop

%Once algorithm has run The optimal path is generated by starting of at the
%last node(if it is the target node) and then identifying its parent node
%until it reaches the start node.This is the optimal path

i = size(CLOSED,1);
Optimal_path = [];
xval = CLOSED(i,1);
yval = CLOSED(i,2);
zval = CLOSED(i,3);
i = 1;
Optimal_path(i,1) = xval;
Optimal_path(i,2) = yval;
Optimal_path(i,3) = zval;
i = i+1;

if ( (xval == xTarget) && (yval == yTarget) && (zval == zTarget))
    inode = 0;
   %Traverse OPEN and determine the parent nodes
   parent_x = OPEN(node_index(OPEN,xval,yval,zval),5);%node_index returns the index of the node 
   parent_y = OPEN(node_index(OPEN,xval,yval,zval),6);
   parent_z = OPEN(node_index(OPEN,xval,yval,zval),7);
   while( parent_x ~= xStart || parent_y ~= yStart || parent_z ~= zStart)
           Optimal_path(i,1) = parent_x;
           Optimal_path(i,2) = parent_y;
           Optimal_path(i,3) = parent_z;
           %Get the grandparents:-)
           inode=node_index(OPEN,parent_x,parent_y,parent_z);
           parent_x = OPEN(inode,5);%node_index returns the index of the node
           parent_y = OPEN(inode,6);
           parent_z = OPEN(inode,7);
           i = i+1;
    end;
    temp = size(Optimal_path);
    Optimal_path(temp(1)+1,1) = xStart;
    Optimal_path(temp(1)+1,2) = yStart;
    Optimal_path(temp(1)+1,3) = zStart;
 j = size(Optimal_path,1);
 %Plot the Optimal Path!
 p = plot3(Optimal_path(j,1)+.5,Optimal_path(j,2)+.5,Optimal_path(j,3)+.5,'bo');
 j = j-1;
 for i = j:-1:1
  pause(.25);
  set(p,'XData',Optimal_path(i,1)+.5,'YData',Optimal_path(i,2)+.5,'ZData',Optimal_path(i,3)+.5);
 drawnow ;
 end;
 
% plot3(Optimal_path(:,1)+.5,Optimal_path(:,2)+.5,Optimal_path(:,3)+.5);
 
 a = [];
 b = [];
 c = [];
 a = Optimal_path(:,1)+.5;
 b = Optimal_path(:,2)+.5;
 c = Optimal_path(:,3)+.5;
 plot3(a,b,c,'b');
 hold on;
 
 
% aa = a';
% bb = b';
% cc = c';
% 
% values = spcrv([[aa(1) aa aa(end)];[bb(1) bb bb(end)];[cc(1) cc cc(end)]],3);
% 
% plot3(values(1,:),values(2,:),values(3,:), 'g');
 
else
 pause(1);
 h=msgbox('Sorry, No path exists to the Target!','warn');
 uiwait(h,5);
end


