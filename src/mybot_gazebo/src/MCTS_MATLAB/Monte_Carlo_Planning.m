function [Robot_Next,Target_Next,Robot_Visibility_Region_Next] = Monte_Carlo_Planning(Initial_Robot,Initial_Target,Robot_Visibility_Region)
%Robustness constant
epsilon = 0.000000001;


%Snap distance (distance within which an observer location will be snapped to the
%boundary before the visibility polygon is computed)
snap_distance = 0.05;


%Read environment geometry from file
environment = read_vertices_from_file('./MCTS1.environment');


%Calculate a good plot window (bounding box) based on outer polygon of environment
environment_min_x = min(environment{1}(:,1));
environment_max_x = max(environment{1}(:,1));
environment_min_y = min(environment{1}(:,2));
environment_max_y = max(environment{1}(:,2));
X_MIN = environment_min_x-0.1*(environment_max_x-environment_min_x);
X_MAX = environment_max_x+0.1*(environment_max_x-environment_min_x);
Y_MIN = environment_min_y-0.1*(environment_max_y-environment_min_y);
Y_MAX = environment_max_y+0.1*(environment_max_y-environment_min_y);

% Initial_Robot = [8;7];
% Initial_Target = [20;20];


detected_time = 0;



%Compute and plot visibility polygon for the target
V{1} = visibility_polygon( [Initial_Robot(1) Initial_Robot(2)] , environment , epsilon , snap_distance );
Area = polyarea(V{1}(:,1),V{1}(:,2));
Reward = Area;
Total_scan = false(1000,1000);
reward_step = 0;
% 
% Robot_Visibility_Region = poly2mask(V{1}(:,1),V{1}(:,2),50, 50);




Negtive_Reward = 30;
%     Total_scan = false(1000,1000);
%Root node
Monte_Carlo = digraph([1],[]);
Monte_Carlo.Nodes.Robot_x= Initial_Robot(1);
Monte_Carlo.Nodes.Robot_y= Initial_Robot(2);
Monte_Carlo.Nodes.Target_x=Initial_Target(1);
Monte_Carlo.Nodes.Target_y=Initial_Target(2);
Monte_Carlo.Nodes.Generation = 1;
Monte_Carlo.Nodes.Parent = 0;
Monte_Carlo.Nodes.Robot_Region{1} = Robot_Visibility_Region;
Monte_Carlo.Nodes.Robot_Reward = bwarea(Monte_Carlo.Nodes.Robot_Region{1});
Monte_Carlo.Nodes.Visited_Time = 1;
Monte_Carlo.Nodes.Total_Reward = 0;
Monte_Carlo.Nodes.Detection_time = 0;
Monte_Carlo.Nodes.UCB_Value = 99999;
Monte_Carlo.Nodes.Average = 0;
Count = 1;
Total_Visited = 1;
Terminal_level = 3;
Plan_level = 17;



% for i = 1 :7783
Monte_Carlo = rmedge(Monte_Carlo,1,1);
while 1
    %pause(0.1)
    v = 1;
    %%selection
    [Monte_Carlo,v] = Monte_Carlo_Selection(Monte_Carlo,v);
    %% expand or rollout
    if Monte_Carlo.Nodes.Generation(v) ~= Terminal_level && Monte_Carlo.Nodes.Visited_Time(v) ~= 0
        Monte_Carlo = Monte_Carlo_Expand(Monte_Carlo,v,environment);
    else %2. Start Rollout Simulation
        roll_node = v;
        Rollout_Robot_Position = [ Monte_Carlo.Nodes.Robot_x(roll_node); Monte_Carlo.Nodes.Robot_y(roll_node) ];
        Rollout_Target_Position = [ Monte_Carlo.Nodes.Target_x(roll_node); Monte_Carlo.Nodes.Target_y(roll_node) ];
        Rollout_Time_step = Plan_level - fix(Monte_Carlo.Nodes.Generation(roll_node)/2);
        Monte_Carlo_Reward = Monte_Carlo_Rollout(Rollout_Robot_Position, Rollout_Target_Position, Monte_Carlo.Nodes.Detection_time(roll_node),Rollout_Time_step,Monte_Carlo.Nodes.Robot_Region{roll_node}, environment, Negtive_Reward);
        %% 3. Backpro pagation
        Monte_Carlo.Nodes.Total_Reward(roll_node) = Monte_Carlo.Nodes.Total_Reward(roll_node) + Monte_Carlo_Reward;
        Monte_Carlo.Nodes.Visited_Time(roll_node) = Monte_Carlo.Nodes.Visited_Time(roll_node) + 1;
        Backpropagation_node = roll_node;
        while Monte_Carlo.Nodes.Parent(Backpropagation_node) ~= 0
            Backpropagation_node = Monte_Carlo.Nodes.Parent(Backpropagation_node);
            Monte_Carlo.Nodes.Total_Reward(Backpropagation_node) = Monte_Carlo.Nodes.Total_Reward(Backpropagation_node) + Monte_Carlo_Reward;
            Monte_Carlo.Nodes.Visited_Time(Backpropagation_node) = Monte_Carlo.Nodes.Visited_Time(Backpropagation_node) + 1;
        end
    end
    
    msg_robot1 = rossubscriber('robot1/odom');
    SIZE_MESSAGE1 = size(msg_robot1.LatestMessage);
    msg_robot2 = rossubscriber('robot2/odom');
    SIZE_MESSAGE2 = size(msg_robot2.LatestMessage);
    if SIZE_MESSAGE1(1) == 0 || SIZE_MESSAGE2(1) == 0
        continue
    else
        robot1_x = msg_robot1.LatestMessage.Pose.Pose.Position.X;
        robot1_y = msg_robot1.LatestMessage.Pose.Pose.Position.Y;
        
        robot2_x = msg_robot2.LatestMessage.Pose.Pose.Position.X;
        robot2_y = msg_robot2.LatestMessage.Pose.Pose.Position.Y;
        
        if norm([robot1_x,robot1_y]-[Initial_Robot(1),Initial_Robot(2)]) <= 0.3 && norm([robot2_x,robot2_y]-[Initial_Target(1),Initial_Target(2)])  <= 0.3
            break
        end
    end

    
end

sucIDs = successors(Monte_Carlo,1);
max_UCB = max(Monte_Carlo.Nodes.Average(sucIDs));
ID = find(Monte_Carlo.Nodes.Average(sucIDs) >= max_UCB-0.001);
Robot_Next = [Monte_Carlo.Nodes.Robot_x(sucIDs(ID)); Monte_Carlo.Nodes.Robot_y(sucIDs(ID))];
Robot_Visibility_Region_Next = Monte_Carlo.Nodes.Robot_Region{sucIDs(ID)};

sucIDs = successors(Monte_Carlo,sucIDs(ID));
min_UCB = min(Monte_Carlo.Nodes.Average(sucIDs));
ID = find(Monte_Carlo.Nodes.Average(sucIDs) <= min_UCB+0.001);
Target_Next = [Monte_Carlo.Nodes.Target_x(sucIDs(ID)); Monte_Carlo.Nodes.Target_y(sucIDs(ID))];
%     Initial_Target = [sensor_x(step+1); sensor_y(step+1)];



