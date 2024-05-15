function volts = voltage_from_currents(conts, vels, Is, Is_dot)
    r_wheel = conts.r_wheel;
    K = conts.K;
    L = conts.L;
    R = conts.R;



    VEL_TO_OMEGA = 1 / r_wheel;

    A = [
        1/L * [-K*VEL_TO_OMEGA, 0, -R, 0];
        1/L * [0, -K*VEL_TO_OMEGA, 0, -R];
    ];
    B = [
        1/L, 0;
        0, 1/L;
    ];

    volts = B \ (Is_dot - A*[vels; Is;]);
end