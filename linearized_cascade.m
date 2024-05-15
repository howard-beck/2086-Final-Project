function u = linearized_cascade(conts, t, state, state_d, V_nom, K_low)
    constants;

    disp(t);

    robot_radius = conts.robot_radius;

    theta = state(THETA);

    Ac = ones(3, 3)*0.0001;
    Bc = [
        cos(theta)/2, cos(theta)/2;
        sin(theta)/2, sin(theta)/2;
        1/2*robot_radius, -1/2*robot_radius
    ];

    Qv = diag([Q_X, Q_Y, Q_THETA]);
    Rv = diag([Q_VEL_RIGHT, Q_VEL_LEFT]);

    [~, Kv, ~] = icare(Ac, Bc, Qv, Rv);

    vs_cmd = state_d(VEL_RIGHT:VEL_LEFT, :) - Kv * (state(X:THETA) - state_d(X:THETA));

    u = V_nom - K_low * (state(VEL_RIGHT:I_LEFT) - [vs_cmd;state_d(I_RIGHT:I_LEFT)]);
end