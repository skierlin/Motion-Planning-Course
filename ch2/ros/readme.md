# ROS 作业结果展示

## 算法的流程与运行结果

### 算法流程：
- demo_node.cpp调用 _astar_path_finder->AstarGraphSearch(start_pt,target_pt) , 规划函数接收起始点和终止点

- 算法主要内容：

 1.  如果 openlist 为空，或者 到达目标点，结束循环
 2. 从openlist中得到f值最小的节点，弹出并标记为已访问过的
 3. 得到当前节点的邻居节点
 4. 决定是否要把邻居节点放到openlist或者改变openlist的g值

- 循环结束后，通过最终的节点，不断回溯，最后得到路径

### 运行结果
![pic1](/home/lance/Desktop/ch2_hw/ros/pic/Selection_048.png)


## 对比不同启发式函数对Astar运行效率的影响

具体实现见函数体内部～

- 对于Manhattan
Time in A* is 0.297402 ms, path cost is 5.024191 m, visited_nodes size : 22

- 对于Euclidean
Time in A* is 42.324343 ms, path cost is 4.789877 m, visited_nodes size : 13242

- 对于Diagonal Heuristic
Time in A* is 43.626084 ms, path cost is 4.789877 m, visited_nodes size : 4642


## 对比是否添加tie breaker对Astar算法的影响

可以进一步减少探索的节点


