clear all; close all; clc;
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


%Initial Information MCTS initial position
% Initial_Robot = [9;6];
% Initial_Target = [40;20];

% %Initial Information ARC_1 initial position 1
% Initial_Robot = [5;7];
% Initial_Target = [20;18];

% %Initial Information ARC_1 initial position 2
Initial_Robot = [8;7];
Initial_Target = [20;20];

% sensor_x =  [20,20,20,20,20,20,20,20,20,19,18,18,18,18,18,17,16,16,17,17,16];
% sensor_y =  [18,17,16,15,14,13,12,11,10,10,10,9,8,7,8,8,8,9,9,8,8];

Robot_path = Initial_Robot;
Opponent_path = Initial_Target;

detected_time = 0;
%Clear plot and form window with desired properties

axis equal; axis off; axis([X_MIN X_MAX Y_MIN Y_MAX]);
hold on;


%Plot environment
patch( environment{1}(:,1) , environment{1}(:,2) , 0.1*ones(1,length(environment{1}(:,1)) ) , ...
    'w' , 'linewidth' , 1.5 );
for i = 2 : size(environment,2)
    patch( environment{i}(:,1) , environment{i}(:,2) , 0.1*ones(1,length(environment{i}(:,1)) ) , ...
        'k' , 'EdgeColor' , [0 0 0] , 'FaceColor' , [0.8 0.8 0.8] , 'linewidth' , 1.5 );
end


%             Plot observer
plot3( Initial_Robot(1) , Initial_Robot(2) , 0.3 , ...
        'o' , 'Markersize' , 15 , 'MarkerEdgeColor' , 'k' , 'MarkerFaceColor' , 'r' );
plot3( Initial_Target(1) , Initial_Target(2), 0.3 , ...
        's' , 'Markersize' , 15, 'MarkerFaceColor' , [0.9,0.8,0.7],'MarkerFaceColor','b','MarkerEdgeColor','b' );
hold on

%Compute and plot visibility polygon
W{1} = visibility_polygon( [Initial_Target(1) Initial_Target(2)] , environment , epsilon , snap_distance );

patch( W{1}(:,1) , W{1}(:,2) , 0.1*ones( size(W{1},1) , 1 ) , ...
    'y' , 'linewidth' , 0.1 );
plot3( W{1}(:,1) , W{1}(:,2) , 0.1*ones( size(W{1},1) , 1 ) , ...
    'b*' , 'Markersize' , 5 );



%Compute and plot visibility polygon for the target
V{1} = visibility_polygon( [Initial_Robot(1) Initial_Robot(2)] , environment , epsilon , snap_distance );
Area = polyarea(V{1}(:,1),V{1}(:,2));
Reward = Area;
Total_scan = false(1000,1000);
reward_step = 0;

patch( V{1}(:,1) , V{1}(:,2) , 0.1*ones( size(V{1},1) , 1 ) , ...
    [0.9,0.6,0.6] , 'linewidth' , 0.1 );
plot3( V{1}(:,1) , V{1}(:,2) , 0.1*ones( size(V{1},1) , 1 ) , ...
    'b*' , 'Markersize' , 5 );
hold off

Robot_Visibility_Region = poly2mask(V{1}(:,1),V{1}(:,2),50, 50);




for step = 1:30
    
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

    
    tic
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
            %         if mod(Monte_Carlo.Nodes.Generation(roll_node),2)
            %             Monte_Carlo.Nodes.UCB_Value(roll_node) = UCBmin_function(Monte_Carlo.Nodes.Total_Reward(roll_node), Monte_Carlo.Nodes.Visited_Time(roll_node), Monte_Carlo.Nodes.Visited_Time(1));
            %         else
            %             Monte_Carlo.Nodes.UCB_Value(roll_node) = UCBmax_function(Monte_Carlo.Nodes.Total_Reward(roll_node), Monte_Carlo.Nodes.Visited_Time(roll_node), Monte_Carlo.Nodes.Visited_Time(1));
            %         end
            %
            
            Backpropagation_node = roll_node;
            while Monte_Carlo.Nodes.Parent(Backpropagation_node) ~= 0
                Backpropagation_node = Monte_Carlo.Nodes.Parent(Backpropagation_node);
                Monte_Carlo.Nodes.Total_Reward(Backpropagation_node) = Monte_Carlo.Nodes.Total_Reward(Backpropagation_node) + Monte_Carlo_Reward;
                Monte_Carlo.Nodes.Visited_Time(Backpropagation_node) = Monte_Carlo.Nodes.Visited_Time(Backpropagation_node) + 1;
                %             if mod(Monte_Carlo.Nodes.Generation(Backpropagation_node),2)
                %                 Monte_Carlo.Nodes.UCB_Value(Backpropagation_node) = UCBmin_function(Monte_Carlo.Nodes.Total_Reward(Backpropagation_node), Monte_Carlo.Nodes.Visited_Time(Backpropagation_node), Monte_Carlo.Nodes.Visited_Time(1));
                %             else
                %                 Monte_Carlo.Nodes.UCB_Value(Backpropagation_node) = UCBmax_function(Monte_Carlo.Nodes.Total_Reward(Backpropagation_node), Monte_Carlo.Nodes.Visited_Time(Backpropagation_node), Monte_Carlo.Nodes.Visited_Time(1));
                %             end
            end
        end
        
        %     for j = 1:nnz(Monte_Carlo.Nodes.Robot_x)
        %         if mod(Monte_Carlo.Nodes.Generation(j),2)
        %             Monte_Carlo.Nodes.UCB_Value(j) = UCBmin_function(Monte_Carlo.Nodes.Total_Reward(j), Monte_Carlo.Nodes.Visited_Time(j), Monte_Carlo.Nodes.Visited_Time(1));
        %         else
        %             Monte_Carlo.Nodes.UCB_Value(j) = UCBmax_function(Monte_Carlo.Nodes.Total_Reward(j), Monte_Carlo.Nodes.Visited_Time(j), Monte_Carlo.Nodes.Visited_Time(1));
        %         end
        %     end
        t=toc;
        if t > 60
            break
        end
        
    end
    
    sucIDs = successors(Monte_Carlo,1);
    max_UCB = max(Monte_Carlo.Nodes.Average(sucIDs));
    ID = find(Monte_Carlo.Nodes.Average(sucIDs) >= max_UCB-0.001);
    Initial_Robot = [Monte_Carlo.Nodes.Robot_x(sucIDs(ID)); Monte_Carlo.Nodes.Robot_y(sucIDs(ID))];
    Robot_Visibility_Region = Monte_Carlo.Nodes.Robot_Region{sucIDs(ID)};
    
    sucIDs = successors(Monte_Carlo,sucIDs(ID));
    min_UCB = min(Monte_Carlo.Nodes.Average(sucIDs));
    ID = find(Monte_Carlo.Nodes.Average(sucIDs) <= min_UCB+0.001);
    Initial_Target = [Monte_Carlo.Nodes.Target_x(sucIDs(ID)); Monte_Carlo.Nodes.Target_y(sucIDs(ID))];
%     Initial_Target = [sensor_x(step+1); sensor_y(step+1)];
    
    Robot_path(:,step+1) = Initial_Robot;
    Opponent_path(:,step+1) = Initial_Target;
    
    W{1} = visibility_polygon( [Initial_Target(1) Initial_Target(2)] , environment , epsilon , snap_distance );
    if in_environment( [Initial_Robot(1), Initial_Robot(2)] , W , epsilon )
        detected_time(step+1) =   1;
    else
        detected_time(step+1) =  0;
    end
%      Opponent_path(:,step+1) = [sensor_x(step+1); sensor_y(step+1)];
    
    
    %Clear plot and form window with desired properties

    axis equal; axis off; axis([X_MIN X_MAX Y_MIN Y_MAX]);
    %Plot environment
    patch( environment{1}(:,1) , environment{1}(:,2) , 0.1*ones(1,length(environment{1}(:,1)) ) , ...
        'w' , 'linewidth' , 1.5 );
    hold on
    for i = 2 : size(environment,2)
        patch( environment{i}(:,1) , environment{i}(:,2) , 0.1*ones(1,length(environment{i}(:,1)) ) , ...
            'k' , 'EdgeColor' , [0 0 0] , 'FaceColor' , [0.8 0.8 0.8] , 'linewidth' , 1.5 );
    end
    
    
    %             Plot observer
    plot3( Initial_Robot(1) , Initial_Robot(2) , 0.3 , ...
        'o' , 'Markersize' , 15 , 'MarkerEdgeColor' , 'k' , 'MarkerFaceColor' , 'r' );
    plot3( Initial_Target(1) , Initial_Target(2), 0.3 , ...
        's' , 'Markersize' , 15, 'MarkerFaceColor' , [0.9,0.8,0.7],'MarkerFaceColor','b','MarkerEdgeColor','b' );
    hold on
    
    %Compute and plot visibility polygon
    W{1} = visibility_polygon( [Initial_Target(1) Initial_Target(2)] , environment , epsilon , snap_distance );
    
    patch( W{1}(:,1) , W{1}(:,2) , 0.1*ones( size(W{1},1) , 1 ) , ...
        'y' , 'linewidth' , 0.1 );
    plot3( W{1}(:,1) , W{1}(:,2) , 0.1*ones( size(W{1},1) , 1 ) , ...
        'b*' , 'Markersize' , 5 );
    
    
    
    %Compute and plot visibility polygon for the target
    V{1} = visibility_polygon( [Initial_Robot(1) Initial_Robot(2)] , environment , epsilon , snap_distance );
    x1= V{1}(:,1);
    y1= V{1}(:,2);
    b1 = poly2mask(x1,y1,1000, 1000);
    Total_scan = b1 | Total_scan;
    Reward(step+1) = bwarea(Total_scan) - Negtive_Reward*sum(detected_time);
    
    patch( V{1}(:,1) , V{1}(:,2) , 0.1*ones( size(V{1},1) , 1 ) , ...
        [0.9,0.6,0.6] , 'linewidth' , 0.1 );
    plot3( V{1}(:,1) , V{1}(:,2) , 0.1*ones( size(V{1},1) , 1 ) , ...
        'b*' , 'Markersize' , 5 );
pause(1)
hold off
end