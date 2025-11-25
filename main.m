function main_extended()
    % Extended UR10e FK + IK + Velocity Kinematics + Trajectory Simulator
    % This version integrates all new functionality with the original program    
    % --- 1. SETUP ---
    clc;
    clear;
    close all;
    
    % ==== Your DH for UR10e (in [a, alpha, d, theta] format) ====
    DH_params = [
        0,       pi/2,   0.181,  0;      % Link 1
        -0.613,  -pi,    0,      0;      % Link 2
        -0.571,  pi,     0,      0;      % Link 3
        0,       -pi/2,  0,      0;      % Link 4
        0,       pi/2,   0,      0;      % Link 5
        -0.12,   0,      0.117,  0;      % Link 6
    ];
    
    % Also create Peter Corke links for URDF visualization only
    L1 = Link('d', 0.181,  'a', 0,       'alpha',  pi/2, 'offset', 0);
    L2 = Link('d', 0,      'a', -0.613,  'alpha', -pi,   'offset', 0);
    L3 = Link('d', 0,      'a', -0.571,  'alpha',  pi,   'offset', 0);
    L4 = Link('d', 0,      'a', 0,       'alpha', -pi/2, 'offset', 0);
    L5 = Link('d', 0,      'a', 0,       'alpha',  pi/2, 'offset', 0);
    L6 = Link('d', 0.117,  'a', -0.12,   'alpha',  0,    'offset', 0);

    UR10e = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'UR10e_DH');

    % --- URDF loading (for visualization) ---
    robot_urdf_file = 'ur10e.urdf';
    ee_link_name = 'link6_1';
    
    try
        robot = importrobot(robot_urdf_file);
        robot.DataFormat = 'column';
    catch ME
        errordlg(['Failed to load robot: ' ME.message], 'File Error');
        return;
    end
    
    disp('----------------------------------------------------');
    disp('>>> VISUALIZATION Robot (from URDF) Body Names:');
    disp(robot.BodyNames);
    disp('----------------------------------------------------');
    
    numJoints = 6;
    
    % URDF joint limits
    jointLimits = zeros(numJoints, 2);
    for i = 1:numJoints
        jointLimits(i,:) = robot.Bodies{i+1}.Joint.PositionLimits;
    end
    
    % --- OFFSETS ---
    dhHome      = [0, -pi/2, 0, pi/2, 0, -pi/2];
    jointOffset = [0, -pi/2, 0, pi/2, 0, -pi/2];
    urdfStart   = dhHome - jointOffset;  % -> zeros
    
    % --- 2. GUI LAYOUT ---
    fig = figure('Name', 'UR10e FK + IK + Velocity Kinematics Simulator', ...
                 'Position', [80, 80, 1400, 800], ...
                 'Units', 'pixels');
    handles.stopFlag = false;
    guidata(fig, handles);

    % LEFT: 3D view
    ax = axes('Parent', fig, ...
              'Units', 'normalized', ...
              'Position', [0.04, 0.05, 0.5, 0.9]);
    
    % RIGHT: control panel
    ctrl = uipanel('Parent', fig, 'Title', 'Control Panel', ...
                   'Units', 'normalized', ...
                   'Position', [0.57, 0.05, 0.4, 0.9], ...
                   'FontSize', 11);
    % Gi? s? fig là figure chính c?a b?n
    logoAxes = axes('Parent', fig, ...
                'Units', 'normalized', ...
                'Position', [-0.025, 0.85, 0.15, 0.15]); % x,y,width,height

    % ??c logo
    logoImg = imread('BK_LOGO.png');

    % Hi?n th? logo
    imshow(logoImg, 'Parent', logoAxes);

    % T?t tr?c
    axis(logoAxes, 'off');

    % FK text (top)
    fkTextHandle = uicontrol('Style', 'text', 'Parent', ctrl, ...
                           'Units', 'normalized', ...
                           'Position', [0.03, 0.945, 0.94, 0.045], ...
                           'String', 'Move sliders to calculate pose...', ...
                           'HorizontalAlignment', 'left', ...
                           'FontSize', 9, ...
                           'FontWeight', 'bold');
    
    % Singularity indicator
    singularityText = uicontrol('Style', 'text', 'Parent', ctrl, ...
                                'Units', 'normalized', ...
                                'Position', [0.03, 0.915, 0.94, 0.025], ...
                                'String', 'Singularity: Checking...', ...
                                'HorizontalAlignment', 'left', ...
                                'FontSize', 8, ...
                                'ForegroundColor', [0 0.5 0], ...
                                'FontWeight', 'bold');
    
    % Store handles in struct
    gui.slider = gobjects(1, numJoints);
    gui.edit   = gobjects(1, numJoints);
    gui.label  = gobjects(1, numJoints);
    
    % Joint controls
    baseY = 0.87;
    stepY = 0.065;
    
    for i = 1:numJoints
        y = baseY - (i-1)*stepY;
        
        gui.label(i) = uicontrol('Style', 'text', 'Parent', ctrl, ...
                           'Units', 'normalized', ...
                           'Position', [0.03, y+0.02, 0.06, 0.03], ...
                           'String', sprintf('J%d', i), ...
                           'HorizontalAlignment', 'left', ...
                           'FontSize', 8);
        
        gui.slider(i) = uicontrol('Style', 'slider', 'Parent', ctrl, ...
                           'Units', 'normalized', ...
                           'Position', [0.10, y+0.015, 0.55, 0.03], ...
                           'Min', jointLimits(i,1), ...
                           'Max', jointLimits(i,2), ...
                           'Value', urdfStart(i));
        
        gui.edit(i) = uicontrol('Style', 'edit', 'Parent', ctrl, ...
                           'Units', 'normalized', ...
                           'Position', [0.67, y+0.015, 0.28, 0.03], ...
                           'String', sprintf('%.4f', urdfStart(i)), ...
                           'HorizontalAlignment', 'center', ...
                           'FontSize', 8);
    end
    
    % --- Velocity Kinematics Panel ---
    velPanel = uipanel('Parent', ctrl, 'Title', 'Velocity Kinematics', ...
                       'Units', 'normalized', ...
                       'Position', [0.03, 0.415, 0.94, 0.14], ...
                       'FontSize', 9, ...
                       'FontWeight', 'bold');
    
    gui.q_dot_edits = gobjects(1, 6);
    
    for i = 1:6
        row = ceil(i/2);
        col = mod(i-1, 2) + 1;
        
        x_pos = 0.02 + (col-1)*0.50;
        y_pos = 0.78 - (row-1)*0.23;
        
        uicontrol('Style', 'text', 'Parent', velPanel, ...
                  'Units', 'normalized', ...
                  'Position', [x_pos, y_pos, 0.13, 0.15], ...
                  'String', sprintf('q%d:', i), ...
                  'HorizontalAlignment', 'left', ...
                  'FontSize', 8);
        
        gui.q_dot_edits(i) = uicontrol('Style', 'edit', 'Parent', velPanel, ...
                  'Units', 'normalized', ...
                  'Position', [x_pos+0.14, y_pos+0.01, 0.30, 0.14], ...
                  'String', '0.0', ...
                  'HorizontalAlignment', 'center', ...
                  'FontSize', 8);
    end
    
    % Velocity result display
    velResult = uicontrol('Style', 'text', 'Parent', ctrl, ...
                              'Units', 'normalized', ...
                              'Position', [0.03, 0.355, 0.94, 0.055], ...
                              'String', 'Velocity:', ...
                              'HorizontalAlignment', 'left', ...
                              'FontSize', 8, ...
                              'FontWeight', 'bold');
    
    gui.velResult = velResult;
    gui.singularityText = singularityText;
    
    % Compute velocity button
    uicontrol('Style', 'pushbutton', 'Parent', velPanel, ...
              'Units', 'normalized', ...
              'Position', [0.05, 0.02, 0.90, 0.25], ...
              'String', 'Compute Velocity', ...
              'FontSize', 8, ...
              'FontWeight', 'bold', ...
              'Callback', @(src,evt) computeVelocityCallback(gui, jointOffset, DH_params, singularityText, velResult));
          
    % --- IK panel ---
    ikPanel = uipanel('Parent', ctrl, 'Title', 'Inverse Kinematics', ...
                      'Units', 'normalized', ...
                      'Position', [0.03, 0.185, 0.94, 0.16], ...
                      'FontSize', 9, ...
                      'FontWeight', 'bold');
    
    uicontrol('Style', 'text', 'Parent', ikPanel, ...
              'Units', 'normalized', ...
              'Position', [0.02, 0.73, 0.10, 0.15], ...
              'String', 'X (m):', 'HorizontalAlignment', 'left', 'FontSize', 7);
    ikX = uicontrol('Style', 'edit', 'Parent', ikPanel, ...
              'Units', 'normalized', ...
              'Position', [0.12, 0.75, 0.18, 0.15], ...
              'String', '0.7', 'FontSize', 8);
    
    uicontrol('Style', 'text', 'Parent', ikPanel, ...
              'Units', 'normalized', ...
              'Position', [0.32, 0.73, 0.10, 0.15], ...
              'String', 'Y (m):', 'HorizontalAlignment', 'left', 'FontSize', 7);
    ikY = uicontrol('Style', 'edit', 'Parent', ikPanel, ...
              'Units', 'normalized', ...
              'Position', [0.42, 0.75, 0.18, 0.15], ...
              'String', '0.2', 'FontSize', 8);
    
    uicontrol('Style', 'text', 'Parent', ikPanel, ...
              'Units', 'normalized', ...
              'Position', [0.62, 0.73, 0.10, 0.15], ...
              'String', 'Z (m):', 'HorizontalAlignment', 'left', 'FontSize', 7);
    ikZ = uicontrol('Style', 'edit', 'Parent', ikPanel, ...
              'Units', 'normalized', ...
              'Position', [0.72, 0.75, 0.18, 0.15], ...
              'String', '0.5', 'FontSize', 8);
    
    uicontrol('Style', 'text', 'Parent', ikPanel, ...
              'Units', 'normalized', ...
              'Position', [0.02, 0.43, 0.10, 0.15], ...
              'String', 'R (rad):', 'HorizontalAlignment', 'left', 'FontSize', 7);
    ikR = uicontrol('Style', 'edit', 'Parent', ikPanel, ...
              'Units', 'normalized', ...
              'Position', [0.12, 0.45, 0.18, 0.15], ...
              'String', '0', 'FontSize', 8);
    
    uicontrol('Style', 'text', 'Parent', ikPanel, ...
              'Units', 'normalized', ...
              'Position', [0.32, 0.43, 0.10, 0.15], ...
              'String', 'P (rad):', 'HorizontalAlignment', 'left', 'FontSize', 7);
    ikP = uicontrol('Style', 'edit', 'Parent', ikPanel, ...
              'Units', 'normalized', ...
              'Position', [0.42, 0.45, 0.18, 0.15], ...
              'String', '0', 'FontSize', 8);
    
    uicontrol('Style', 'text', 'Parent', ikPanel, ...
              'Units', 'normalized', ...
              'Position', [0.62, 0.43, 0.10, 0.15], ...
              'String', 'Y (rad):', 'HorizontalAlignment', 'left', 'FontSize', 7);
    ikYaw = uicontrol('Style', 'edit', 'Parent', ikPanel, ...
              'Units', 'normalized', ...
              'Position', [0.72, 0.45, 0.18, 0.15], ...
              'String', '0', 'FontSize', 8);
    
    uicontrol('Style', 'pushbutton', 'Parent', ikPanel, ...
              'Units', 'normalized', ...
              'Position', [0.05, 0.10, 0.90, 0.25], ...
              'String', 'Solve IK & Apply', ...
              'FontSize', 8, ...
              'FontWeight', 'bold', ...
              'Callback', @(src,evt) solveIKAndApply(robot, UR10e, gui, fkTextHandle, ax, jointOffset, ee_link_name, ikX, ikY, ikZ, ikR, ikP, ikYaw, DH_params, singularityText));
    
    % --- Trajectory Planning Panel ---
    trajPanel = uipanel('Parent', ctrl, 'Title', 'Trajectory Planning', ...
                        'Units', 'normalized', ...
                        'Position', [0.03, 0.01, 0.94, 0.17], ...
                        'FontSize', 9, ...
                        'FontWeight', 'bold');
    
    % Circle Path button
    uicontrol('Style', 'pushbutton', 'Parent', trajPanel, ...
          'Units', 'normalized', ...
          'Position', [0.05, 0.65, 0.42, 0.28], ...
          'String', 'Circle Path', ...
          'FontSize', 8, ...
          'Callback', @(src,evt) executeCirclePath(robot, UR10e, ax, DH_params, jointOffset, gui, fkTextHandle, singularityText));

    % Square Path button
    uicontrol('Style', 'pushbutton', 'Parent', trajPanel, ...
          'Units', 'normalized', ...
          'Position', [0.53, 0.65, 0.42, 0.28], ...
          'String', 'Square Path', ...
          'FontSize', 8, ...
          'Callback', @(src,evt) executeSquarePath(robot, UR10e, ax, DH_params, jointOffset, gui, fkTextHandle, singularityText));

    % Line Path button
    uicontrol('Style', 'pushbutton', 'Parent', trajPanel, ...
          'Units', 'normalized', ...
          'Position', [0.05, 0.32, 0.42, 0.28], ...
          'String', 'Line Path', ...
          'FontSize', 8, ...
          'Callback', @(src,evt) executeLinePath(robot, UR10e, ax, DH_params, jointOffset, gui, fkTextHandle, singularityText));

    % Figure-8 Path button
    uicontrol('Style', 'pushbutton', 'Parent', trajPanel, ...
          'Units', 'normalized', ...
          'Position', [0.53, 0.32, 0.42, 0.28], ...
          'String', 'Figure-8 Path', ...
          'FontSize', 8, ...
          'Callback', @(src,evt) executeFigure8Path(robot, UR10e, ax, DH_params, jointOffset, gui, fkTextHandle, singularityText));
    
    % Trong ph?n Trajectory Planning Panel, thay dòng này:
    uicontrol('Style','pushbutton','Parent',trajPanel, ...
          'Units','normalized','Position',[0.05,0.01,0.90,0.25], ...
          'String','Show Workspace','FontSize',8,'FontWeight','bold', ...
          'Callback', @(src,evt) plotWorkspaceRobot(robot, UR10e, jointLimits, jointOffset, ax, ee_link_name, DH_params, gui, fkTextHandle, singularityText));
      
%     uicontrol('Style','pushbutton','Parent',trajPanel, ...
%           'Units','normalized','Position',[0.15,0.03,0.22,0.10], ...
%           'String','STOP','FontSize',10,'FontWeight','bold', ...
%           'BackgroundColor',[1 0.6 0.6], ...
%           'Callback', @(src,evt) stopCallback(fig));
% 
%     uicontrol('Style','pushbutton','Parent',trajPanel, ...
%           'Units','normalized','Position',[0.653,0.03,0.22,0.10], ...
%           'String','CLEAR','FontSize',10,'FontWeight','bold', ...
%           'BackgroundColor',[0.8 0.85 1], ...
%           'Callback', @(src,evt) clearDrawing(ax));

    % --- connect slider + edit callbacks ---
    for i = 1:numJoints
        set(gui.slider(i), 'Callback', @(s,e) sliderChanged(i, gui, robot, UR10e, fkTextHandle, ax, jointOffset, ee_link_name, DH_params, singularityText));
        set(gui.edit(i),   'Callback', @(s,e) editChanged(i, gui, robot, UR10e, fkTextHandle, ax, jointOffset, ee_link_name, jointLimits, DH_params, singularityText));
    end
    
    % --- initial plot ---
    updateFromURDFq(robot, UR10e, fkTextHandle, ax, urdfStart(:), jointOffset, ee_link_name, DH_params, singularityText);
end


%% ==================== CALLBACKS ====================

function sliderChanged(idx, gui, robot, UR10e, fkTextHandle, ax, jointOffset, ee_link_name, DH_params, singularityText)
    q_urdf = readAllURDF(gui);
    set(gui.edit(idx), 'String', sprintf('%.4f', q_urdf(idx)));
    updateFromURDFq(robot, UR10e, fkTextHandle, ax, q_urdf, jointOffset, ee_link_name, DH_params, singularityText);
end

function editChanged(idx, gui, robot, UR10e, fkTextHandle, ax, jointOffset, ee_link_name, jointLimits, DH_params, singularityText)
    val = str2double(get(gui.edit(idx), 'String'));
    if isnan(val)
        val = get(gui.slider(idx), 'Value');
        set(gui.edit(idx), 'String', sprintf('%.4f', val));
    end
    val = max(min(val, jointLimits(idx,2)), jointLimits(idx,1));
    set(gui.slider(idx), 'Value', val);
    q_urdf = readAllURDF(gui);
    updateFromURDFq(robot, UR10e, fkTextHandle, ax, q_urdf, jointOffset, ee_link_name, DH_params, singularityText);
end

function q_urdf = readAllURDF(gui)
    n = numel(gui.slider);
    q_urdf = zeros(n,1);
    for k = 1:n
        q_urdf(k) = get(gui.slider(k), 'Value');
    end
end

function updateFromURDFq(robot, UR10e, fkTextHandle, ax, q_urdf, jointOffset, ee_link_name, DH_params, singularityText)
    q_dh = q_urdf' + jointOffset;
    
    DH_current = DH_params;
    DH_current(:,4) = q_dh';
    
    [T_custom, ~] = FK(DH_current);
    
    pos = T_custom(1:3, 4)';
    rpy = tr2rpy(T_custom);
    
    poseString1 = sprintf('Pos [X,Y,Z]: %.3f, %.3f, %.3f', pos(1), pos(2), pos(3));
    poseString2 = sprintf('RPY [R,P,Y]: %.2f, %.2f, %.2f', rpy(1), rpy(2), rpy(3));
    set(fkTextHandle, 'String', {poseString1, poseString2});
    
    [is_singular, det_val, sing_type] = detectSingularity(DH_current, 0.01);
    
    if is_singular
        sing_string = sprintf('SINGULARITY: %s (det=%.4f)', sing_type, det_val);
        set(singularityText, 'String', sing_string, 'ForegroundColor', [1 0 0]);
    else
        sing_string = sprintf('OK (det(J) = %.4f)', det_val);
        set(singularityText, 'String', sing_string, 'ForegroundColor', [0 0.5 0]);
    end
    
    show(robot, q_urdf, 'Parent', ax, 'PreservePlot', false);
    title(ax, 'Robot Visualization');
    axis(ax, 'on'); grid(ax, 'on');
    drawnow;
end


%% ==================== HELPER FUNCTIONS ====================

function [T, A] = FK(DH)
    n = size(DH,1);
    A = cell(1,n);
    T = eye(4);
    for i = 1:n
        a = DH(i,1);
        alpha = DH(i,2);
        d = DH(i,3);
        theta = DH(i,4);
        Ai = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
              sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
              0 sin(alpha) cos(alpha) d;
              0 0 0 1];
        A{i} = Ai;
        T = T * Ai;
    end
end

function solutions = IK_UR10e(T_des)
    a     = [ 0,   -0.613,  -0.571,   0,     0,    -0.12 ];
    alpha = [ pi/2,  -pi,     pi,   -pi/2,  pi/2,   0     ];
    d     = [ 0.181,  0,       0,     0,     0,    0.117 ];

    p_des = T_des(1:3,4);
    R_des = T_des(1:3,1:3);

    q = [0; -pi/2; 0; pi/2; 0; -pi/2];

    maxIter   = 800;
    lambda    = 0.01;
    tol_pos   = 1e-4;
    tol_ori   = 1e-3;
    maxStep   = 0.2;

    n = 6;

    for iter = 1:maxIter
        DH = [a(:), alpha(:), d(:), q(:)];
        [T_cur, A_cells] = FK(DH);
        p_cur = T_cur(1:3,4);
        R_cur = T_cur(1:3,1:3);

        e_pos = p_des - p_cur;
        R_err = R_des * R_cur';
        e_ori = rotm2axangVec_local(R_err);
        e = [e_pos; e_ori];

        if norm(e_pos) < tol_pos && norm(e_ori) < tol_ori
            break;
        end

        T_i = eye(4);
        p = zeros(3, n+1);
        z = zeros(3, n+1);
        p(:,1) = T_i(1:3,4);
        z(:,1) = T_i(1:3,3);

        for i = 1:n
            T_i = T_i * A_cells{i};
            p(:,i+1) = T_i(1:3,4);
            z(:,i+1) = T_i(1:3,3);
        end

        p_n = p(:,end);
        Jv = zeros(3,n);
        Jw = zeros(3,n);
        for i = 1:n
            Jv(:,i) = cross(z(:,i), p_n - p(:,i));
            Jw(:,i) = z(:,i);
        end
        J = [Jv; Jw];

        dq = (J.'*J + (lambda^2)*eye(n)) \ (J.'*e);
        stepNorm = norm(dq);
        if stepNorm > maxStep
            dq = dq * (maxStep / stepNorm);
        end
        q = q + dq;
    end

    DH_final = [a(:), alpha(:), d(:), q(:)];
    [T_final, ~] = FK(DH_final);
    p_final = T_final(1:3,4);
    R_final = T_final(1:3,1:3);

    pos_err = norm(p_des - p_final);
    ori_err = norm(rotm2axangVec_local(R_des * R_final'));

    if pos_err > 0.05 || ori_err > 0.2
        solutions = [];
    else
        solutions = q.';
    end
end

function w = rotm2axangVec_local(R)
    cos_theta = (trace(R) - 1) / 2;
    cos_theta = max(min(cos_theta, 1), -1);
    theta = acos(cos_theta);

    if abs(theta) < 1e-6
        w = [0;0;0];
        return;
    end

    k = 1/(2*sin(theta)) * [ R(3,2) - R(2,3);
                             R(1,3) - R(3,1);
                             R(2,1) - R(1,2) ];
    w = theta * k;
end

%% UR10e EXTENSION FUNCTIONS
% All velocity kinematics, singularity detection, and trajectory functions
% Place this file in the same directory as main_extended.m

%% ================== JACOBIAN COMPUTATION ==================

function [J, det_J] = computeNumericJacobian(DH_params)
    % Compute numeric Jacobian for given joint angles
    n = 6;
    [T_final, A_cells] = FK(DH_params);
    
    p_e = T_final(1:3, 4);
    J = zeros(6, n);
    
    T_cum = eye(4);
    z_prev = [0; 0; 1];
    o_prev = [0; 0; 0];
    
    J(1:3, 1) = cross(z_prev, p_e - o_prev);
    J(4:6, 1) = z_prev;
    
    for i = 2:n
        T_cum = T_cum * A_cells{i-1};
        z_i = T_cum(1:3, 3);
        o_i = T_cum(1:3, 4);
        
        J(1:3, i) = cross(z_i, p_e - o_i);
        J(4:6, i) = z_i;
    end
    
    det_J = det(J);
end


%% ================== VELOCITY KINEMATICS ==================

function [v_linear, w_angular, J] = computeEndEffectorVelocity(DH_params, q_dot)
    % Compute end-effector velocity given joint velocities
    [J, ~] = computeNumericJacobian(DH_params);
    
    V_ee = J * q_dot(:);
    
    v_linear = V_ee(1:3);
    w_angular = V_ee(4:6);
end


%% ================== SINGULARITY DETECTION ==================

function [is_singular, det_val, singularity_type] = detectSingularity(DH_params, threshold)
    % Detect if robot is near a singularity
    if nargin < 2
        threshold = 0.001;
    end
    
    [~, det_val] = computeNumericJacobian(DH_params);
    
    is_singular = abs(det_val) < threshold;
    
    singularity_type = 'None';
    
    if is_singular
        theta = DH_params(:, 4);
        
        if abs(theta(5)) < 0.1 || abs(abs(theta(5)) - pi) < 0.1
            singularity_type = 'Wrist Singularity';
        elseif abs(theta(3)) < 0.1 || abs(abs(theta(3)) - pi) < 0.1
            singularity_type = 'Elbow Singularity ';
        else
            singularity_type = 'Shoulder/Boundary Singularity';
        end
    end
end


%% ================== VELOCITY CALLBACK ==================

function computeVelocityCallback(gui, jointOffset, DH_params, singularityText, velResult)
    % Callback for velocity computation button
    
    q_urdf = readAllURDF(gui);
    q_dh = q_urdf(:) + jointOffset(:);
    
    q_dot = zeros(6, 1);
    for i = 1:6
        val = str2double(get(gui.q_dot_edits(i), 'String'));
        if isnan(val)
            val = 0;
        end
        q_dot(i) = val;
    end
    
    DH_current = DH_params;
    DH_current(:, 4) = q_dh;
    
    [v_linear, w_angular, J] = computeEndEffectorVelocity(DH_current, q_dot);
    
    [is_singular, det_val, sing_type] = detectSingularity(DH_current, 0.01);
    
    vel_string = sprintf(['Linear Velocity [vx, vy, vz]: [%.4f, %.4f, %.4f] m/s\n', ...
                         'Angular Velocity [wx, wy, wz]: [%.4f, %.4f, %.4f] rad/s\n', ...
                         'det(J) = %.6f'], ...
                         v_linear(1), v_linear(2), v_linear(3), ...
                         w_angular(1), w_angular(2), w_angular(3), det_val);
    
    set(velResult, 'String', vel_string);
    
    if is_singular
        sing_string = sprintf('WARNING: SINGULARITY DETECTED! (%s)', sing_type);
        set(singularityText, 'String', sing_string, 'ForegroundColor', [1 0 0]);
    else
        sing_string = sprintf('Configuration OK (det(J) = %.4f)', det_val);
        set(singularityText, 'String', sing_string, 'ForegroundColor', [0 0.5 0]);
    end
    
    fprintf('\n========================================\n');
    fprintf('   VELOCITY KINEMATICS RESULTS\n');
    fprintf('========================================\n');
    fprintf('Joint Velocities (rad/s):\n');
    fprintf('  [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', q_dot);
    fprintf('\nJacobian:\n');
    disp(J);
    fprintf('Determinant: %.6f\n', det_val);
    fprintf('\nEnd-Effector Velocity:\n');
    fprintf('  Linear:  [%.4f, %.4f, %.4f] m/s\n', v_linear);
    fprintf('  Angular: [%.4f, %.4f, %.4f] rad/s\n', w_angular);
    
    if is_singular
        fprintf('\nSINGULARITY WARNING: %s\n', sing_type);
    end
    fprintf('========================================\n\n');
end


%% ================== IK SOLVER AND CALLBACK ==================

function solveIKAndApply(robot, UR10e, gui, fkTextHandle, ax, jointOffset, ee_link_name, ikX, ikY, ikZ, ikR, ikP, ikYaw, DH_params, singularityText)
    % Solve IK and apply solution to robot
    
    x = str2double(get(ikX, 'String'));
    y = str2double(get(ikY, 'String'));
    z = str2double(get(ikZ, 'String'));
    r = str2double(get(ikR, 'String'));
    p = str2double(get(ikP, 'String'));
    yw = str2double(get(ikYaw, 'String'));
    
    if any(isnan([x y z r p yw]))
        errordlg('IK input contains non-numeric values','IK Error');
        return;
    end
    
    target_pos = [x; y; z];
    reach_distance = norm(target_pos);
    
    max_reach = 0.613 + 0.571;
    min_reach = 0.10;
    z_max = 1.3;
    z_min = -0.3;
    
    fprintf('\n========================================\n');
    fprintf('        IK REACHABILITY CHECK\n');
    fprintf('========================================\n');
    fprintf('Target Position: [%.3f, %.3f, %.3f] m\n', x, y, z);
    fprintf('Distance from base: %.3f m\n', reach_distance);
    fprintf('Workspace limits:\n');
    fprintf('  - Max reach: ~%.3f m\n', max_reach);
    fprintf('  - Min reach: ~%.3f m\n', min_reach);
    fprintf('  - Z range: [%.2f, %.2f] m\n', z_min, z_max);
    
    if reach_distance > max_reach
        errordlg(sprintf('Target UNREACHABLE! Distance %.3f m exceeds max %.3f m', ...
                        reach_distance, max_reach), 'Out of Reach');
        fprintf('FAILED: Target exceeds maximum reach\n');
        fprintf('========================================\n\n');
        return;
    end
    
    if reach_distance < min_reach
        errordlg(sprintf('Target TOO CLOSE! Distance %.3f m < min %.3f m', ...
                        reach_distance, min_reach), 'Too Close');
        fprintf('FAILED: Target is too close to base\n');
        fprintf('========================================\n\n');
        return;
    end
    
    if z > z_max || z < z_min
        errordlg(sprintf('Height %.3f m out of range [%.2f, %.2f]', ...
                        z, z_min, z_max), 'Height Error');
        fprintf('FAILED: Target height out of range\n');
        fprintf('========================================\n\n');
        return;
    end
    
    fprintf('Target is within workspace\n');
    fprintf('========================================\n\n');
    
    Tgoal = transl(x, y, z) * rpy2tr(r, p, yw);
    
    fprintf('Solving IK...\n');
    
    all_solutions = IK_UR10e(Tgoal);
    
    if isempty(all_solutions)
        errordlg('IK failed! No solution found.', 'IK Failed');
        fprintf('IK solver: No solutions\n\n');
        return;
    end
    
    fprintf('IK solved successfully\n');
    
    q_dh_sol = all_solutions(1, :);
    
    DH_sol = DH_params;
    DH_sol(:,4) = q_dh_sol';
    T_verify = FK(DH_sol);
    
    pos_actual = T_verify(1:3, 4);
    pos_error = norm(target_pos - pos_actual);
    
    fprintf('Position error: %.4f m (%.1f mm)\n', pos_error, pos_error*1000);
    
    if pos_error > 0.05
        warndlg(sprintf('Large error: %.1f mm', pos_error*1000), 'IK Warning');
    end
    
    fprintf('========================================\n\n');
    
    q_urdf_sol = (q_dh_sol - jointOffset);
    
    for i = 1:numel(gui.slider)
        set(gui.slider(i), 'Value', q_urdf_sol(i));
        set(gui.edit(i), 'String', sprintf('%.4f', q_urdf_sol(i)));
    end
    
    updateFromURDFq(robot, UR10e, fkTextHandle, ax, q_urdf_sol', jointOffset, ee_link_name, DH_params, singularityText);
end


%% ================== TRAJECTORY GENERATION ==================

function [q_trajectory, t_trajectory] = generateTrajectory(waypoints_cart, waypoints_rpy, DH_params, jointOffset, dt)
    % Generate smooth joint-space trajectory through Cartesian waypoints
    
    if nargin < 5
        dt = 0.05;
    end
    
    N_waypoints = size(waypoints_cart, 1);
    
    q_waypoints = zeros(N_waypoints, 6);
    
    fprintf('\n========================================\n');
    fprintf('   TRAJECTORY PLANNING\n');
    fprintf('========================================\n');
    fprintf('Number of waypoints: %d\n\n', N_waypoints);
    
    for i = 1:N_waypoints
        T_goal = transl(waypoints_cart(i, 1), waypoints_cart(i, 2), waypoints_cart(i, 3)) * ...
                 rpy2tr(waypoints_rpy(i, 1), waypoints_rpy(i, 2), waypoints_rpy(i, 3));
        
        all_solutions = IK_UR10e(T_goal);
        
        if isempty(all_solutions)
            error('IK failed for waypoint %d: [%.3f, %.3f, %.3f]', ...
                  i, waypoints_cart(i, 1), waypoints_cart(i, 2), waypoints_cart(i, 3));
        end
        
        if i == 1
            q_waypoints(i, :) = all_solutions(1, :);
        else
            distances = sum((all_solutions - q_waypoints(i-1, :)).^2, 2);
            [~, best_idx] = min(distances);
            q_waypoints(i, :) = all_solutions(best_idx, :);
        end
        
        fprintf('Waypoint %d: IK solved ?\n', i);
    end
    
    fprintf('\nGenerating smooth trajectory...\n');
    
    segment_time = 1.0;
    total_time = (N_waypoints - 1) * segment_time;
    t_trajectory = 0:dt:total_time;
    N_samples = length(t_trajectory);
    
    q_trajectory = zeros(N_samples, 6);
    waypoint_times = linspace(0, total_time, N_waypoints);
    
    for j = 1:6
        q_trajectory(:, j) = interp1(waypoint_times, q_waypoints(:, j), ...
                                      t_trajectory, 'pchip');
    end
    
    fprintf('Trajectory generated: %d samples over %.2f seconds\n', N_samples, total_time);
    fprintf('========================================\n\n');
end


%% ================== TRAJECTORY ANIMATION ==================

%% ================== TRAJECTORY ANIMATION (FIXED) ==================

function animateTrajectory(robot, UR10e, q_trajectory, t_trajectory, ax, jointOffset, DH_params, gui, fkTextHandle, singularityText)
    % Animate robot following trajectory with FULL GUI updates
    
    N = size(q_trajectory, 1);
    
    fprintf('\n========================================\n');
    fprintf('   ANIMATING TRAJECTORY\n');
%     fprintf('========================================\n');
%     fprintf('Total samples: %d\n', N);
%     fprintf('Duration: %.2f seconds\n', t_trajectory(end));
%     fprintf('Press Ctrl+C to stop animation\n');
%     fprintf('========================================\n\n');

    ee_path = zeros(N, 3);
    
    for i = 1:N
        q_dh = q_trajectory(i, :);
        q_urdf = q_dh - jointOffset;
        % === TÍNH FORWARD KINEMATICS ===
        DH_current = DH_params;
        DH_current(:, 4) = q_dh';
        [T_current, ~] = FK(DH_current);
        
        pos = T_current(1:3, 4)';
        rpy = tr2rpy(T_current);
        ee_path(i, :) = pos;
        % === C?P NH?T SLIDERS & EDIT BOXES ===
        if mod(i, 10) == 0 || i == 1 || i == N
            if ~isempty(gui)
                for j = 1:6
                    set(gui.slider(j), 'Value', q_urdf(j));
                    set(gui.edit(j), 'String', sprintf('%.4f', q_urdf(j)));
                end
            end
        end
        
        % === C?P NH?T FK TEXT (Position & Orientation) ===
        if ~isempty(fkTextHandle) && isvalid(fkTextHandle)
            poseString1 = sprintf('Pos [X,Y,Z]: %.3f, %.3f, %.3f', pos(1), pos(2), pos(3));
            poseString2 = sprintf('RPY [R,P,Y]: %.2f, %.2f, %.2f', rpy(1), rpy(2), rpy(3));
            set(fkTextHandle, 'String', {poseString1, poseString2});
        end
        
        % === KI?M TRA SINGULARITY ===
        if ~isempty(singularityText) && isvalid(singularityText)
            [is_singular, det_val, sing_type] = detectSingularity(DH_current, 0.01);
            
            if is_singular
                sing_string = sprintf('SINGULARITY: %s (det=%.4f)', sing_type, det_val);
                set(singularityText, 'String', sing_string, 'ForegroundColor', [1 0 0]);
            else
                sing_string = sprintf('OK (det(J) = %.4f)', det_val);
                set(singularityText, 'String', sing_string, 'ForegroundColor', [0 0.5 0]);
            end
        end
        
        % === V? ROBOT ===
        show(robot, q_urdf', 'Parent', ax, 'PreservePlot', false);
        hold(ax, 'on');
        
        % === V? TRAJECTORY PATH ===
        if i > 1
            plot3(ax, ee_path(1:i, 1), ee_path(1:i, 2), ee_path(1:i, 3), ...
                  'r-', 'LineWidth', 2);
        end
        
        % === V? END-EFFECTOR MARKER ===
        plot3(ax, ee_path(i, 1), ee_path(i, 2), ee_path(i, 3), ...
              'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
        
        title(ax, sprintf('Trajectory Animation'));
        grid(ax, 'on');
        axis(ax, 'on');
        
        % C?p nh?t màn hình
        drawnow limitrate;
    end
    
    hold(ax, 'off');
    fprintf('\nAnimation complete!\n\n');
end


%% ================== C?P NH?T CÁC HÀM TRAJECTORY EXECUTION ==================

function executeCirclePath(robot, UR10e, ax, DH_params, jointOffset, gui, fkTextHandle, singularityText)
    % Execute circular trajectory with GUI updates
    
    center = [0.5, 0.2, 0.5];
    radius = 0.15;
    N_points = 10;
    
    theta = linspace(0, 2*pi, N_points);
    waypoints_cart = zeros(N_points, 3);
    waypoints_rpy = zeros(N_points, 3);
    
    for i = 1:N_points
        waypoints_cart(i, :) = center + [0, radius*cos(theta(i)), radius*sin(theta(i))];
        waypoints_rpy(i, :) = [0, pi/2, 0];
    end
    
    fprintf('\n>>> Generating CIRCLE trajectory...\n');
    fprintf('Center: [%.3f, %.3f, %.3f] m, Radius: %.3f m\n', center, radius);
    
    try
        [q_traj, t_traj] = generateTrajectory(waypoints_cart, waypoints_rpy, ...
                                               DH_params, jointOffset, 0.05);
        animateTrajectory(robot, UR10e, q_traj, t_traj, ax, jointOffset, ...
                         DH_params, gui, fkTextHandle, singularityText);
    catch ME
        errordlg(['Trajectory execution failed: ' ME.message], 'Error');
    end
end

function executeSquarePath(robot, UR10e, ax, DH_params, jointOffset, gui, fkTextHandle, singularityText)
    % Execute square trajectory with GUI updates
    
    center = [0.6, 0.0, 0.4];
    side_length = 0.2;
    N_per_side = 3;
    
    X_fixed = center(1);
    half = side_length / 2;
    corners_YZ = [
        center(2) - half, center(3) - half;
        center(2) + half, center(3) - half;
        center(2) + half, center(3) + half;
        center(2) - half, center(3) + half;
        center(2) - half, center(3) - half;
    ];

    waypoints_cart = [];
    for i = 1:4
        edge_points = linspace(0, 1, N_per_side)';
        edge_waypoints_YZ = corners_YZ(i,:) + edge_points .* (corners_YZ(i+1,:) - corners_YZ(i,:));
        edge_waypoints = [X_fixed*ones(size(edge_waypoints_YZ,1),1), edge_waypoints_YZ];
        waypoints_cart = [waypoints_cart; edge_waypoints];
    end
    
    waypoints_rpy = repmat([0, pi/2, 0], size(waypoints_cart, 1), 1);
    
    fprintf('\n>>> Generating SQUARE trajectory...\n');
    fprintf('Center: [%.3f, %.3f, %.3f] m, Side: %.3f m\n', center, side_length);
    
    try
        [q_traj, t_traj] = generateTrajectory(waypoints_cart, waypoints_rpy, ...
                                               DH_params, jointOffset, 0.05);
        animateTrajectory(robot, UR10e, q_traj, t_traj, ax, jointOffset, ...
                         DH_params, gui, fkTextHandle, singularityText);
    catch ME
        errordlg(['Trajectory execution failed: ' ME.message], 'Error');
    end
end

function executeLinePath(robot, UR10e, ax, DH_params, jointOffset, gui, fkTextHandle, singularityText)
    % Execute linear trajectory with GUI updates
    
    start_point = [0.4, -0.2, 0.3];
    end_point = [0.7, 0.3, 0.6];
    N_points = 10;
    
    t = linspace(0, 1, N_points)';
    waypoints_cart = [start_point(1)*ones(size(t)), ...
                      start_point(2) + t*(end_point(2)-start_point(2)), ...
                      start_point(3) + t*(end_point(3)-start_point(3))];

    waypoints_rpy = repmat([0, pi/2, 0], N_points, 1);
    
    fprintf('\n>>> Generating LINEAR trajectory...\n');
    fprintf('From [%.3f, %.3f, %.3f] to [%.3f, %.3f, %.3f] m\n', ...
            start_point, end_point);
    
    try
        [q_traj, t_traj] = generateTrajectory(waypoints_cart, waypoints_rpy, ...
                                               DH_params, jointOffset, 0.05);
        animateTrajectory(robot, UR10e, q_traj, t_traj, ax, jointOffset, ...
                         DH_params, gui, fkTextHandle, singularityText);
    catch ME
        errordlg(['Trajectory execution failed: ' ME.message], 'Error');
    end
end

function executeFigure8Path(robot, UR10e, ax, DH_params, jointOffset, gui, fkTextHandle, singularityText)
    % Execute figure-8 trajectory with GUI updates
    
    center = [0.55, 0.0, 0.5];
    scale = 0.12;
    N_points = 20;
    
    t = linspace(0, 2*pi, N_points);
    waypoints_cart = zeros(N_points, 3);
    
    X_fixed = center(1);
    for i = 1:N_points
        y = scale * cos(t(i));
        z = scale * sin(t(i)) * cos(t(i));
        waypoints_cart(i, :) = [X_fixed, center(2) + y, center(3) + z];
    end
    
    waypoints_rpy = repmat([0, pi/2, 0], N_points, 1);
    
    fprintf('\n>>> Generating FIGURE-8 trajectory...\n');
    fprintf('Center: [%.3f, %.3f, %.3f] m, Scale: %.3f m\n', center, scale);
    
    try
        [q_traj, t_traj] = generateTrajectory(waypoints_cart, waypoints_rpy, ...
                                               DH_params, jointOffset, 0.05);
        animateTrajectory(robot, UR10e, q_traj, t_traj, ax, jointOffset, ...
                         DH_params, gui, fkTextHandle, singularityText);
    catch ME
        errordlg(['Trajectory execution failed: ' ME.message], 'Error');
    end
end

function plotWorkspaceRobot(robot, UR10e, jointLimits, jointOffset, ax, ee_link_name, DH_params, gui, fkTextHandle, singularityText)
    % Plot workspace WITH ROBOT ANIMATION - RANDOM SAMPLING (both normal & singular)
    
    % --- L?y figure & handles ---
    fig = ancestor(ax, 'figure');
    handles = guidata(fig);
    
    if ~isvalid(ax)
        errordlg('Axes không h?p l?. Không th? v? workspace.', 'Error');
        return;
    end
    
    % Reset stop flag
    handles.stopFlag = false;
    guidata(fig, handles);
    
    % --- T?o workspace group ---
    if ~isfield(handles,'wsGroup') || ~isvalid(handles.wsGroup)
        handles.wsGroup = hggroup(ax);
    else
        % Xóa workspace c? n?u có
        delete(handles.wsGroup);
        handles.wsGroup = hggroup(ax);
    end
    guidata(fig, handles);
    
    % --- CÀI ??T SAMPLING ---
    N_samples = 500;  % S? ?i?m random (thay vì 729 ?i?m grid)
    nJoints = 6;
    
    % === RANDOM SAMPLING trong joint limits ===
    q_all = zeros(N_samples, nJoints);
    for i = 1:nJoints
        q_all(:, i) = jointLimits(i,1) + (jointLimits(i,2) - jointLimits(i,1)) * rand(N_samples, 1);
    end
    
    total_points = N_samples;
    
    hold(ax, 'on');
    colormap(ax, jet);
    
    % --- THÔNG BÁO B?T ??U ---
    fprintf('\n========================================\n');
    fprintf('   WORKSPACE EXPLORATION - RANDOM SAMPLING\n');
    fprintf('========================================\n');
    fprintf('Total random configurations: %d\n', total_points);
    fprintf('Robot will visit each point...\n');
    fprintf('Press STOP button to interrupt\n');
    fprintf('========================================\n\n');
    
    % --- ROBOT CH?Y QUA T?NG ?I?M ---
    singular_count = 0;
    normal_count = 0;
    
    % === BI?N ??M ?? GI?M T?N SU?T C?P NH?T GUI ===
    gui_update_interval = 5;  % Ch? update GUI m?i 5 ?i?m
    
    for k = 1:total_points
        % === KI?M TRA STOP FLAG ===
        handles = guidata(fig);
        if handles.stopFlag
            fprintf('\n>>> STOPPED by user at point %d/%d\n', k-1, total_points);
            handles.stopFlag = false;
            guidata(fig, handles);
            break;
        end
        
        % === L?Y GÓC KH?P HI?N T?I ===
        q_dh = q_all(k, :);
        q_urdf = q_dh - jointOffset(:)';
        
        % === TÍNH FORWARD KINEMATICS ===
        DH_current = DH_params;
        DH_current(:, 4) = q_dh';
        [T_current, ~] = FK(DH_current);
        
        pos = T_current(1:3, 4)';
        rpy = tr2rpy(T_current);
        
        % === KI?M TRA SINGULARITY ===
        [is_singular, det_val, sing_type] = detectSingularity(DH_current, 0.005);
        
        % ??m s? l??ng
        if is_singular
            singular_count = singular_count + 1;
            point_status = 'SINGULAR';
            status_color = [1 0 0];  % ??
            marker_color = 'r';
            marker_size = 35;
        else
            normal_count = normal_count + 1;
            point_status = 'NORMAL';
            status_color = [0 0.5 0];  % Xanh lá
            marker_color = pos(3);  % Màu theo Z
            marker_size = 25;
        end
        
        % === C?P NH?T GUI CH? M?I gui_update_interval ?I?M ===
        % ?i?u này gi?m lag c?a sliders
        if mod(k, gui_update_interval) == 0 || k == 1 || k == total_points
            % C?P NH?T SLIDERS & EDIT BOXES
            if ~isempty(gui)
                for j = 1:6
                    set(gui.slider(j), 'Value', q_urdf(j));
                    set(gui.edit(j), 'String', sprintf('%.4f', q_urdf(j)));
                end
            end
            
            % C?P NH?T FK TEXT
            if ~isempty(fkTextHandle) && isvalid(fkTextHandle)
                poseString1 = sprintf('Pos [X,Y,Z]: %.3f, %.3f, %.3f m', pos(1), pos(2), pos(3));
                poseString2 = sprintf('RPY [R,P,Y]: %.2f, %.2f, %.2f rad', rpy(1), rpy(2), rpy(3));
                poseString3 = sprintf('Status: %s', point_status);
                set(fkTextHandle, 'String', {poseString1, poseString2, poseString3});
            end
            
            % C?P NH?T SINGULARITY TEXT
            if ~isempty(singularityText) && isvalid(singularityText)
                if is_singular
                    sing_string = sprintf('SINGULARITY: %s | det(J)=%.6f', sing_type, det_val);
                else
                    sing_string = sprintf('OK | det(J)=%.6f', det_val);
                end
                set(singularityText, 'String', sing_string, 'ForegroundColor', status_color);
            end
            
            % V? ROBOT (ch? update m?i gui_update_interval ?? tránh lag)
            show(robot, q_urdf', 'Parent', ax, 'PreservePlot', false);
            hold(ax, 'on');
        end
        
        % === V? ?I?M WORKSPACE (V? T?T C? ?I?M, KHÔNG PHÂN BI?T) ===
        if is_singular
            % ?i?m k? d?: ?? v?i vi?n ?en
            scatter3(pos(1), pos(2), pos(3), ...
                marker_size, 'r', 'filled', 'Parent', handles.wsGroup, ...
                'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
        else
            % ?i?m bình th??ng: màu theo Z (GRADIENT)
            scatter3(pos(1), pos(2), pos(3), ...
                marker_size, marker_color, 'filled', 'Parent', handles.wsGroup);
        end
        
        % === C?P NH?T TITLE ===
        if mod(k, gui_update_interval) == 0 || k == 1 || k == total_points
            progress_pct = (k / total_points) * 100;
            title(ax, sprintf(['Workspace Point %d/%d (%.1f%%) | Normal: %d | Singular: %d\n' ...
                              'Current: [%.3f, %.3f, %.3f] - %s'], ...
                             k, total_points, progress_pct, normal_count, singular_count, ...
                             pos(1), pos(2), pos(3), point_status), ...
                  'FontSize', 9);
            
            grid(ax, 'on');
            axis(ax, 'on');
            xlabel(ax, 'X (m)');
            ylabel(ax, 'Y (m)');
            zlabel(ax, 'Z (m)');
        end
        
        % === IN CONSOLE ===
        if mod(k, 50) == 0 || k == 1 || is_singular
            fprintf('[%4d/%d] Pos:[%6.3f,%6.3f,%6.3f] RPY:[%5.2f,%5.2f,%5.2f] det(J):%8.5f %s\n', ...
                    k, total_points, pos(1), pos(2), pos(3), ...
                    rpy(1), rpy(2), rpy(3), det_val, point_status);
        end
        
        % === C?P NH?T MÀN HÌNH ===
        if mod(k, gui_update_interval) == 0 || k == 1 || k == total_points
            drawnow limitrate;
            
            % Xóa marker hi?n t?i
            if exist('h_current', 'var') && isvalid(h_current)
                delete(h_current);
            end
        end
        
        % Ki?m tra axes còn t?n t?i
        if ~isvalid(ax)
            fprintf('\n>>> Axes deleted, stopping\n');
            return;
        end
    end
    
    % --- K?T THÚC ---
    hold(ax, 'off');
    
    % === HOÀN THI?N ?? H?A ===
    xlabel(ax, 'X (m)');
    ylabel(ax, 'Y (m)');
    zlabel(ax, 'Z (m)');
    title(ax, sprintf('Workspace Complete: %d points (%d normal, %d singular)', ...
                     k, normal_count, singular_count));
    grid(ax, 'on');
    axis(ax, 'equal');
    view(ax, 3);
    
    % Colorbar
    cb = colorbar(ax);
    ylabel(cb, 'Height Z (m)');
    
    % Legend
    if singular_count > 0
        legend(ax, {'Normal Config', 'Singular Config'}, 'Location', 'northeast');
    end
    
    % --- TH?NG KÊ CU?I ---
    fprintf('\n========================================\n');
    fprintf('   WORKSPACE EXPLORATION COMPLETE\n');
    fprintf('========================================\n');
    fprintf('Total points sampled: %d\n', k);
    fprintf('Normal configurations: %d (%.1f%%)\n', normal_count, (normal_count/k)*100);
    fprintf('Singular configurations: %d (%.1f%%)\n', singular_count, (singular_count/k)*100);
    fprintf('========================================\n\n');
    
    % C?p nh?t singularity text cu?i cùng
    if ~isempty(singularityText) && isvalid(singularityText)
        final_string = sprintf('Workspace Done: %d points (%d singular)', k, singular_count);
        set(singularityText, 'String', final_string, 'ForegroundColor', [0 0.5 0]);
    end
end
function stopCallback(fig)
    handles = guidata(fig);
    handles.stopFlag = true;
    guidata(fig, handles);
    disp(">>> STOP requested!");
end
function clearDrawing(ax)
    fig = ancestor(ax,'figure');
    handles = guidata(fig);

    % Stop any ongoing motion
    handles.stopFlag = true;

    % CH? XOÁ NHÓM WORKSPACE
    if isfield(handles,'wsGroup') && isvalid(handles.wsGroup)
        delete(handles.wsGroup);
    end

    % T?o group m?i ?? v? ti?p
    handles.wsGroup = hggroup(ax);
    guidata(fig, handles);

    disp(">>> Workspace cleared (robot preserved).");
end



