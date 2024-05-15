function state_deriv = model_deriv(conts, state, u)
    constants;

    robot_radius = conts.robot_radius;

    theta = state(THETA);
    vel_right = state(VEL_RIGHT);
    vel_left  = state(VEL_LEFT);

    state_deriv = zeros(7, 1);

    state_deriv(X) = (vel_right + vel_left) / 2 * cos(theta);
    state_deriv(Y) = (vel_right + vel_left) / 2 * sin(theta);
    state_deriv(THETA) = (vel_right - vel_left) / (2 * robot_radius);

    [A_low, B_low] = motor_deriv_matrix(conts);

    state_deriv(VEL_RIGHT:I_LEFT) = A_low * state(VEL_RIGHT:I_LEFT) + B_low * u;
end