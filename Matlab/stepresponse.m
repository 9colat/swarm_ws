function stepresponse(number)
fileNumber = string(number); %the input file number
filename = 'log'+fileNumber+'.txt'; %import the file
delimiterIn = ','; %specify the separator
headerlinesIn = 1;
dataTable = importdata(filename,delimiterIn,headerlinesIn); %import

RightWheelSpeed = dataTable.data(:,1); %make a column vector for the first column from the table
LeftWheelSpeed = dataTable.data(:,2); %and so on
InputOmegaRight = dataTable.data(:,3);
InputLinearSpeed = dataTable.data(:,4);



x = (0:1:size(RightWheelSpeed,1)-1).';

f1 = figure('Name','Right Wheel','NumberTitle','off');
plot(x,RightWheelSpeed)
hold on
plot(x,InputOmegaRight);
grid on
title('Right Wheel Angular Velocity over Time')
xlabel('Time [s]'); %remember to specify this correctly
ylabel('Angular Velocity [rad/s]');
legend({'Right Wheel Speed Output', 'Right Wheel Speed Input'},'Position',[0.67 0.18 0.1 0.02])
hold off


f2 = figure('Name','Left Wheel','NumberTitle','off');
plot(x,LeftWheelSpeed);
hold on
plot(x,InputOmegaRight);
grid on
title('Left Wheel Angular Velocity over Time')
xlabel('Time [s]'); %remember to specify this correctly
ylabel('Angular Velocity [rad/s]');
legend({'Left Wheel Speed Output', 'Left Wheel Speed Input'},'Position',[0.67 0.18 0.1 0.02])
hold off

 
end