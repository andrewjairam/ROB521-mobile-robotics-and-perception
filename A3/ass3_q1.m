% =========
% ass3_q1.m
% =========
%
% This assignment will introduce you to the idea of first building an
% occupancy grid then using that grid to estimate a robot's motion using a
% particle filter.
% 
% There are two questions to complete (5 marks each):
%
%    Question 1: code occupancy mapping algorithm 
%    Question 2: see ass3_q2.m
%
% Fill in the required sections of this script with your code, run it to
% generate the requested plot/movie, then paste the plots into a short report
% that includes a few comments about what you've observed.  Append your
% version of this script to the report.  Hand in the report as a PDF file
% and the two resulting AVI files from Questions 1 and 2.
%
% requires: basic Matlab, 'gazebo.mat'
%
% T D Barfoot, January 2016
%
clear all;

% set random seed for repeatability
rng(1);

% ==========================
% load the dataset from file
% ==========================
%
%    ground truth poses: t_true x_true y_true theta_true
% odometry measurements: t_odom v_odom omega_odom
%           laser scans: t_laser y_laser
%    laser range limits: r_min_laser r_max_laser
%    laser angle limits: phi_min_laser phi_max_laser
%
load gazebo.mat;

% =======================================
% Question 1: build an occupancy grid map
% =======================================
%
% Write an occupancy grid mapping algorithm that builds the map from the
% perfect ground-truth localization.  Some of the setup is done for you
% below.  The resulting map should look like "ass2_q1_soln.png".  You can
% watch the movie "ass2_q1_soln.mp4" to see what the entire mapping process
% should look like.  At the end you will save your occupancy grid map to
% the file "occmap.mat" for use in Question 2 of this assignment.

% allocate a big 2D array for the occupancy grid
ogres = 0.05;                   % resolution of occ grid
ogxmin = -7;                    % minimum x value
ogxmax = 8;                     % maximum x value
ogymin = -3;                    % minimum y value
ogymax = 6;                     % maximum y value
ognx = (ogxmax-ogxmin)/ogres;   % number of cells in x direction
ogny = (ogymax-ogymin)/ogres;   % number of cells in y direction
oglo = zeros(ogny,ognx);        % occupancy grid in log-odds format
ogp = zeros(ogny,ognx);         % occupancy grid in probability format

% precalculate some quantities
numodom = size(t_odom,1);
npoints = size(y_laser,2);
angles = linspace(phi_min_laser, phi_max_laser,npoints);
dx = ogres*cos(angles);
dy = ogres*sin(angles);

% interpolate the noise-free ground-truth at the laser timestamps
t_interp = linspace(t_true(1),t_true(numodom),numodom);
x_interp = interp1(t_interp,x_true,t_laser);
y_interp = interp1(t_interp,y_true,t_laser);
theta_interp = interp1(t_interp,theta_true,t_laser);
omega_interp = interp1(t_interp,omega_odom,t_laser);
  
% set up the plotting/movie recording
vid = VideoWriter('ass2_q1.avi');
open(vid);
figure(1);
clf;
pcolor(ogp);
colormap(1-gray);
shading('flat');
axis equal;
axis off;
M = getframe;
writeVideo(vid,M);

% loop over laser scans (every fifth)
for i=1:5:size(t_laser,1)

    % ------insert your occupancy grid mapping algorithm here------

    % 1. Initialize to prior likelihood of being occupied: Done above
    % 2. Loop over scans: Loop over all cells in field of view (we are in that loop), update affected cells using equation slide 11 lec24
    % 3. Threshold cells based on whether they became more or less likely than prior

    % Step (0a): Params for grid mapping algorithm: alpha: weight for occupied cells, beta: weight for frre cells. TUNABLE PARAMS
    alpha = 2;
    beta = 1;

    % Step (0b): Start loop over each timestep, get current scans and continue if scans are NaN (no data) or out of the valid range
    for j=1:npoints
        cur_scans = y_laser(i,j);
        cur_laser_angles = angles(j);
        if any(isnan(cur_scans)) || any(cur_scans < r_min_laser) || any(cur_scans > r_max_laser)
            continue;
        end

        % Step (1) First transform the laser scans to the robot frame, to get laser endpoint in robot frame
        x_endpoint = x_interp(i) + cur_scans*cos(theta_interp(i) + cur_laser_angles);
        y_endpoint = y_interp(i) + cur_scans*sin(theta_interp(i) + cur_laser_angles);

        % Step (2) Convert to (integer) image coordinates: 
        % convert both the robot position and the laser endpoint to image coordinates, so we can get all cells between them.
        x_endpt_map = round((x_endpoint - ogxmin)/ogres);
        y_endpt_map = round((y_endpoint - ogymin)/ogres);
        x_robot_map = round((x_interp(i) - ogxmin)/ogres);
        y_robot_map = round((y_interp(i) - ogymin)/ogres);
        % We don't want to clip because we might classify the map bounds as obstacles if we clipped lidar, just skip this iteration instead
        if x_endpt_map < 1 || x_endpt_map > ognx || y_endpt_map < 1 || y_endpt_map > ogny 
            continue;
        end
        if x_robot_map < 1 || x_robot_map > ognx || y_robot_map < 1 || y_robot_map > ogny
            continue;
        end

        % Step (3) Get all cells along the line from the robot to the laser endpoint
        % Easy (and inefficient) way: use linear interpolation: step in x and y by some stepsize
        % Number of steps: approx. with the max of the difference in x and y. This will step in the direction of most change
        num_steps = max(abs(x_endpt_map - x_robot_map), abs(y_endpt_map - y_robot_map));
        x_step = (x_endpt_map - x_robot_map)/num_steps;
        y_step = (y_endpt_map - y_robot_map)/num_steps;
        % Can update the map with cells along the line in this loop: exclude last entry, which is the endpoint
        for k=0:num_steps-1
            x_pixel = round(x_robot_map + k*x_step);
            y_pixel = round(y_robot_map + k*y_step);
            % If within bounds, Update the occupancy grid: 
            if x_pixel >= 1 && x_pixel <= ognx && y_pixel >= 1 && y_pixel <= ogny
                % If the cell is the endpoint, it is occupied
                if i == num_steps-1
                    oglo(y_pixel, x_pixel) = oglo(y_pixel, x_pixel) + alpha;
                % If the cell is not the endpoint, it is free
                else
                    oglo(y_pixel, x_pixel) = oglo(y_pixel, x_pixel) - beta;
                end
            end
        end


        % Step (4) Update the (log odds)occupancy grid: 
        % The endpoint: occupied (hit an obstacle)
        % The cells along the line excluding end point: free (no obstacle)
        % Done above, while finding the cells along the line!

        % Step (5) At the end, convert log odds to probabilities
        ogp = exp(oglo)./(1 + exp(oglo));  

    end
    % ------end of your occupancy grid mapping algorithm-------


    % draw the map
    clf;
    pcolor(ogp);
    colormap(1-gray);
    shading('flat');
    axis equal;
    axis off;
    
    % draw the robot
    hold on;
    x = (x_interp(i)-ogxmin)/ogres;
    y = (y_interp(i)-ogymin)/ogres;
    th = theta_interp(i);
    r = 0.15/ogres;
    set(rectangle( 'Position', [x-r y-r 2*r 2*r], 'Curvature', [1 1]),'LineWidth',2,'FaceColor',[0.35 0.35 0.75]);
    set(plot([x x+r*cos(th)]', [y y+r*sin(th)]', 'k-'),'LineWidth',2);
    
    % save the video frame
    M = getframe;
    writeVideo(vid,M);
    
    pause(0.1);
    
end

close(vid);
print -dpng ass2_q1.png

save occmap.mat ogres ogxmin ogxmax ogymin ogymax ognx ogny oglo ogp;

