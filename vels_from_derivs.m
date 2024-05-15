function [vels] = vels_from_derivs(conts, x_dot, y_dot, theta_d)
    robot_radius = conts.robot_radius;

    v_mag = sqrt(x_dot.^2 + y_dot.^2);

    addition    = v_mag * 2;
    subtraction = 2*robot_radius * theta_d;

    vel_right = 1/2 * (addition + subtraction);
    vel_left  = 1/2 * (addition - subtraction);
    
    vels = [vel_right; vel_left];
end