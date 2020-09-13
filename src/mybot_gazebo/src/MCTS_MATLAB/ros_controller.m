px = 3*[0,1,2,2,3,3,3,4];
py = 3*[1,1,1,2,2,3,4,4];
pub_goal = rospublisher('/goal','nav_msgs/Odometry');
msg = rosmessage('nav_msgs/Odometry');
stage = 0;
msg.Pose.Pose.Position.X = px(1);
msg.Pose.Pose.Position.Y = py(1);
msg.Pose.Pose.Position.Z = 1;
send(pub_goal,msg);

while 1
    msg2 = rossubscriber('/odom');
        SIZE_MESSAGE = size(msg2.LatestMessage);
        if SIZE_MESSAGE(1) == 0
            continue
        else
            rx = msg2.LatestMessage.Pose.Pose.Position.X;
            ry = msg2.LatestMessage.Pose.Pose.Position.Y;
            break
        end
end
 i =1;
while i <= 7   
    msg.Pose.Pose.Position.X = px(i);
    msg.Pose.Pose.Position.Y = py(i);
    msg.Pose.Pose.Position.Z = i;
    tic
    t1=0;
    
    if norm([rx,ry]-[px(i),py(i)]) >= 0.5
        send(pub_goal,msg);
    else
        i = i + 1;
    end

    msg2 = rossubscriber('/odom');
    SIZE_MESSAGE = size(msg2.LatestMessage);
    if SIZE_MESSAGE(1) == 0
        continue
    else
        rx = msg2.LatestMessage.Pose.Pose.Position.X;
        ry = msg2.LatestMessage.Pose.Pose.Position.Y;
    end
    

end
