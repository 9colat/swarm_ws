function vel = swarmLineController(poses,rIdx,detections)
    pose = poses(:,rIdx);
    
    v = 0;
    w = 2;
    
    % Else, turn towards the average of detected robots on the same team
    if ~isempty(detections)
            
        % Take the average range and angle
        range = min(detections(:,1));
        angle = min(detections(:,2));
        
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

    % Convert to global velocity
    vel = bodyToWorld([v;0;w],pose);
end

