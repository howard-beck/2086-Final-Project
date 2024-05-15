function u = full_state_feedback(conts, t, state, state_d, V_nom)
    constants;

    disp(t);

    robot_radius = conts.robot_radius;

    %err = state - state_d;

    %Q = diag(1, 1, 0, 0.5, 0.5);
    %R = diag(1, 1);

    x = state(X);
    y = state(Y);
    theta = state(THETA);
    vel_right = state(VEL_RIGHT);
    vel_left  = state(VEL_LEFT);

    x_dot_nom = model_deriv(conts, state_d, [0;0]);

    % x' = (vel_right + vel_left)/2 * cos(theta)

    v_avg = (vel_right + vel_left)/2;
    % omega = (vel_right - vel_left)/2*r

    Ac = zeros(7, 7);
    Ac(1:3, 1:3) = ones(3, 3)*0.0001;
    Ac(1:3, 4:5) = [
        cos(theta)/2, cos(theta)/2;
        sin(theta)/2, sin(theta)/2;
        1/2*robot_radius, -1/2*robot_radius
    ];

    [A_low, B_low] = motor_deriv_matrix(conts);

    Ac(4:7, 4:7) = A_low;

    Bc = zeros(7, 2);
    Bc(4:7, :) = B_low;

    Q_fsf = diag([Q_X, Q_Y, Q_THETA, Q_VEL_RIGHT, Q_VEL_LEFT, Q_I_RIGHT, Q_I_LEFT]);
    R_fsf = diag([R_V_RIGHT, R_V_LEFT]);

    [~, K_fsf, ~] = icare(Ac, Bc, Q_fsf, R_fsf);
    %%K_low
    %err_here()

    u = V_nom; %- K_fsf * (state - state_d);
end