% Add paths
addpath('./simulation_scripts');
addpath('./tools');
addpath('./icat');
addpath('./robust_robot');
addpath('./tasks/')
clc; clear; close all;

% Simulation parameters
dt       = 0.005;
endTime  = 60;
% Initialize robot model and simulator
robotModel = UvmsModel();          
sim = UvmsSim(dt, robotModel, endTime);
% Initialize Unity interface
unity = UnityInterface("127.0.0.1");

% Define tasks    
task_tool    = TaskTool();
task_vehicle_pos = TaskVehiclePosition();       
task_vehicle_mis = TaskVehicleMisalignment();
task_vehicle_alt = TaskVehicleAltitude();
task_vehicle_land = TaskVehicleLand();

task_set1 = { task_vehicle_alt, task_vehicle_mis, task_vehicle_pos };   % Safe Navigation
task_set2 = { task_vehicle_land, task_vehicle_mis, task_vehicle_pos };  % Landing

% Define actions and add to ActionManager
actionManager = ActionManager();
actionManager.addAction(task_set1);  % action 1: Safe Navigation
actionManager.addAction(task_set2);  % action 2: Landing

% Unifying task list


% Define desired positions and orientations (world frame)
w_arm_goal_position = [12.2025, 37.3748, -39.8860]';
w_arm_goal_orientation = [0, pi, pi/2];
%w_vehicle_goal_position = [10.5, 37.5, -38]';
%w_vehicle_goal_position = [12.2025 37.3748 -39.8860]';
%w_vehicle_goal_position = [12.2025 37.3748 -45]';
w_vehicle_goal_position = [50 -12.5 -33]';
w_vehicle_goal_orientation = [0, 0, 0];

% Set goals in the robot model
robotModel.setGoal(w_arm_goal_position, w_arm_goal_orientation, w_vehicle_goal_position, w_vehicle_goal_orientation);

% Initialize the logger
logger = SimulationLogger(ceil(endTime/dt)+1, robotModel, task_set1);
idx=1;

% Main simulation loop
for step = 1:sim.maxSteps
    % 1. Receive altitude from Unity
    robotModel.altitude = unity.receiveAltitude(robotModel);

    % 2. Compute control commands for current action
    [v_nu, q_dot] = actionManager.computeICAT(robotModel);

    % 3. Step the simulator (integrate velocities)
    sim.step(v_nu, q_dot);

    % 4. Send updated state to Unity
    unity.send(robotModel);

    % 5. Logging
    logger.update(sim.time, sim.loopCounter);

    % 6. Optional debug prints
    if mod(sim.loopCounter, round(1 / sim.dt)) == 0
        fprintf('t = %.2f s\n', sim.time);
        fprintf('alt = %.2f m\n', robotModel.altitude);
    end

    % 7. Optional real-time slowdown
    SlowdownToRealtime(dt);
    
    % Control   
    if(norm(v_nu) < 0.2)
        actionManager.setCurrentAction(2);
    end
   

end

% Display plots
logger.plotAll();

% Clean up Unity interface
delete(unity);