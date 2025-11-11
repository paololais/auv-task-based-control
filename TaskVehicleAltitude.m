%% 
classdef TaskVehicleAltitude < Task   
    properties
        desired_alt = 2.0;  % meters from seafloor
    end

    methods
        function updateReference(obj, robot)
            if isempty(robot.altitude)
                alt = 2;
            else
                alt = robot.altitude;
            end

            error = obj.desired_alt - alt;
                      
            obj.xdotbar = 0.2 * error;
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);

        end

        function updateJacobian(obj, robot)
            % Jacobian maps z-velocity to altitude change
            % [0 0 1] selects z component of linear velocity
            obj.J = [0 0 1] * [zeros(3,7), eye(3), zeros(3,3)];
        end
        
        function updateActivation(obj, robot)
            % Activate when close to seafloor
            % Fully active when altitude < 2.0m
            % Start activating when altitude < 2.5m
            % Compute scalar activation
            if isempty(robot.altitude)
                alt = 2;
            else
                alt = robot.altitude;
            end

            obj.A = DecreasingBellShapedFunction(2.0, 2.5, 0, 1, alt);
        end
    end
end