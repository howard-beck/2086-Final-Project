function currents = currents_from_vels(conts, vels, vels_dot)
    constants;
    
    robot_mass = conts.robot_mass;
    robot_radius = conts.robot_radius;
    robot_MoI = conts.robot_MoI;
    r_wheel = conts.r_wheel;
    b = conts.b;
    K = conts.K;

    

    VEL_TO_OMEGA = 1 / r_wheel;

    Cp = (1/robot_mass + robot_radius^2/robot_MoI);
    Cm = (1/robot_mass - robot_radius^2/robot_MoI);

    A = [
        Cp*G*r_wheel*[-b*VEL_TO_OMEGA, 0] + Cm*G*r_wheel*[0, -b*VEL_TO_OMEGA];
        Cp*G*r_wheel*[0, -b*VEL_TO_OMEGA] + Cm*G*r_wheel*[-b*VEL_TO_OMEGA, 0];
    ];
    B = [
        Cp*G*r_wheel*[K, 0] + Cm*G*r_wheel*[0, K];
        Cp*G*r_wheel*[0, K] + Cm*G*r_wheel*[K, 0];
    ];
    % A vel + B I = vels_dot
    
    currents = B \ (vels_dot - A * vels);
end