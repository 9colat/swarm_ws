function vel = swarmTeamController(poses,rIdx,detections,numTeams)
    
    % Unpack the robot's pose and team index
    pose = poses(:,rIdx);
    teamIdx = mod(rIdx,numTeams);

    % If there are no detections, turn in place
    v = 0;
    w = 2;
    
    % Else, turn towards the average of detected robots on the same team
    if ~isempty(detections)
        validInds = find(mod(detections(:,3),numTeams) == teamIdx);
        if ~isempty(validInds)
            
            % Take the average range and angle
            range = mean(detections(validInds,1));
            angle = mean(detections(validInds,2));
            
            % Move linearly to maintain a range to the nearest robot
            if range > 0.6
                v = 0.5;
            elseif range < 0.4
                v = -0.5;
            end
            
            % Turn to maintain a heading to the nearest robot
            if angle > pi/12
                w = 2;
            elseif angle < -pi/12
                w = -2;
            end
            
        end
        
    end

    % Convert to global velocity
    vel = bodyToWorld([v;0;w],pose);

end