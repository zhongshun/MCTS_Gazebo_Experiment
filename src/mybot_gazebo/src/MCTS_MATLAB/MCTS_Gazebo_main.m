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
V{1} = visibility_polygon( [Initial_Robot1(1) Initial_Robot1(2)] , environment , epsilon , snap_distance );
Area = polyarea(V{1}(:,1),V{1}(:,2));
Reward = Area;
Total_scan = false(1000,1000);
reward_step = 0;
Robot_Visibility_Region = poly2mask(V{1}(:,1),V{1}(:,2),50, 50);

Initial_Robot1 = [8;7];
Initial_Robot2 = [20;20];

pub_goal_robot1 = rospublisher('robot1/goal','nav_msgs/Odometry');
robot1 = rosmessage('nav_msgs/Odometry');
stage = 0;
robot1.Pose.Pose.Position.X = Initial_Robot1(1);
robot1.Pose.Pose.Position.Y = Initial_Robot1(2);
robot1.Pose.Pose.Position.Z = 1;
send(pub_goal_robot1,robot1);

pub_goal_robot2 = rospublisher('robot2/goal','nav_msgs/Odometry');
robot2 = rosmessage('nav_msgs/Odometry');
stage = 0;
robot2.Pose.Pose.Position.X = Initial_Robot2(1);
robot2.Pose.Pose.Position.Y = Initial_Robot2(2);
robot2.Pose.Pose.Position.Z = 1;
send(pub_goal_robot2,robot1);

for step = 2:10
    [Robot1_Next,Robot2_Next,Robot_Visibility_Region_Next] = Monte_Carlo_Planning(Initial_Robot1,Initial_Robot2,Robot_Visibility_Region);
    
    robot1.Pose.Pose.Position.X = Robot1_Next(1);
    robot1.Pose.Pose.Position.Y = Robot1_Next(2);
    robot1.Pose.Pose.Position.Z = 1;
    send(pub_goal_robot1,robot1);
    Initial_Robot1 = Robot1_Next;
    
    robot2.Pose.Pose.Position.X = Robot2_Next(1);
    robot2.Pose.Pose.Position.Y = Robot2_Next(2);
    robot2.Pose.Pose.Position.Z = 1;
    send(pub_goal_robot2,robot2);
    Initial_Robot2 = Robot2_Next;
    
end