close all
rosshutdown
rosinit
sub = rossubscriber('/USPS_pose_estimator','geometry_msgs/Pose')%,'DataFormat','struct');
f1 = figure;
old_x = 0;
old_y = 0;
old_z = 0;

%msg = receive(sub);
%while 1
    msg = receive(sub);
    if msg.Position.X ~= old_x || msg.Position.Y ~= old_y || msg.Position.Z ~= old_z
        rt_plotter(msg)
        old_x = msg.Position.X;
        old_y = msg.Position.Y;
        old_z = msg.Position.Z;
    end
%end



function rt_plotter(data)
    figure(f1);
    ID =        [42867, 42928,  42929,  44530,  44531,  44532,  44533,  44534,  44535,  44536,  44537,  44538,  44540];
    x_coor =    [11700, 16244,  7824,   2000,   21369,  26163,  26163,  31000,  35766,  35766,  40205,  40204,  16560];
    y_coor =    [5999,  10150,  5726,   4499,   6534,   9939,   3699,   6519,   10012,  3522,   11684,  4363,   3549];
    z_coor =    [5577,  5577,   4286,   3530,   5578,   5577,   5577,   5578,   5578,   5578,   3767,   3767,   3767];

    hold on
    for i = 1:size(ID,2)
        plot3(y_coor(i),x_coor(i),z_coor(i),'or');
        text(y_coor(i)+100,x_coor(i),z_coor(i),int2str(ID(i)));
    end
    axis([0 12000 0  45000])
    plot3(data.Position.X,data.Position.Y,data.Position.Z,'.b')
    hold off
end