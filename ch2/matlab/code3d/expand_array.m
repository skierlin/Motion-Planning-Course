function exp_array=expand_array(node_x,node_y,node_z,hn,xTarget,yTarget,zTarget,CLOSED,MAX_X,MAX_Y,MAX_Z)
    %Function to return an expanded array
    %This function takes a node and returns the expanded list of successors,with the calculated fn values.
    %The criteria being none of the successors are on the CLOSED list.
    exp_array=[];
    exp_count=1;
    c2=size(CLOSED,1);%Number of elements in CLOSED including the zeros
    for k= 1:-1:-1
        for j= 1:-1:-1
            for i= 1:-1:-1
                if (k~=j || k~=0 || k~=i || j~=i)  %The node itself is not its successor
                    s_x = node_x+k;
                    s_y = node_y+j;
                    s_z = node_z+i;
                    if( (s_x >0 && s_x <=MAX_X) && (s_y >0 && s_y <=MAX_Y) && (s_z >0 && s_z <=MAX_Z))
                        flag=1;                    
                        for c1=1:c2
                            if(s_x == CLOSED(c1,1) && s_y == CLOSED(c1,2) && s_z == CLOSED(c1,3))
                                flag=0;
                            end;
                        end;%End of for loop to check if a successor is on closed list.
                        if (flag == 1)
                            exp_array(exp_count,1) = s_x;
                            exp_array(exp_count,2) = s_y;
                            exp_array(exp_count,3) = s_z;
                            exp_array(exp_count,4) = hn+distance(node_x,node_y,node_z,s_x,s_y,s_z);%cost of travelling to node
                            exp_array(exp_count,5) = distance(xTarget,yTarget,zTarget,s_x,s_y,s_z);%distance between node and goal
                            exp_array(exp_count,6) = exp_array(exp_count,4)+exp_array(exp_count,5);%fn
                            exp_count=exp_count+1;
                        end%Populate the exp_array list!!!
                        
                end% End of node within array bound
                end
            end%End of if node is not its own successor loop
        end%End of j for loop
    end%End of k for loop    