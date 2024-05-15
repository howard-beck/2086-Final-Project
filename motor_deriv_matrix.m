function [A_low, B_low] = motor_deriv_matrix(conts)
    constants;

    robot_mass = conts.robot_mass;
    robot_radius = conts.robot_radius;
    robot_MoI = conts.robot_MoI;
    r_wheel = conts.r_wheel;
    b = conts.b;
    K = conts.K;
    L = conts.L;
    R = conts.R;



    VEL_TO_OMEGA = 1 / r_wheel;

    Cp = (1/robot_mass + robot_radius^2/robot_MoI);
    Cm = (1/robot_mass - robot_radius^2/robot_MoI);

    A_low = [
        Cp*G*r_wheel*[-b*VEL_TO_OMEGA, 0, K, 0] + Cm*G*r_wheel*[0, -b*VEL_TO_OMEGA, 0, K];
        Cp*G*r_wheel*[0, -b*VEL_TO_OMEGA, 0, K] + Cm*G*r_wheel*[-b*VEL_TO_OMEGA, 0, K, 0];
        1/L * [-K*VEL_TO_OMEGA, 0, -R, 0];
        1/L * [0, -K*VEL_TO_OMEGA, 0, -R];
    ];
    B_low = [
        0, 0;
        0, 0;
        1/L, 0;
        0, 1/L;
    ];
end