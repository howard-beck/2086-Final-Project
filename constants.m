J0 = 0.01;
b0 = 0.1;
K0 = 0.01;
R0 = 1;
L0 = 0.5;
G = 100;




X = 1;
Y = 2;
THETA = 3;
VEL_RIGHT = 4;
VEL_LEFT  = 5;
I_RIGHT = 6;
I_LEFT = 7;

V_RIGHT = 1;
V_LEFT = 2;




in_to_cm = 2.54;
lb_to_kg = 1/2.204623;
r_wheel0 = 2.5 * in_to_cm;
robot_radius0 = 1 * 12 * in_to_cm/100;
robot_mass0 = 75 * lb_to_kg;
robot_MoI0 = 1/3 * robot_mass0 * (0.7 * robot_radius0)^2;

dt = 0.1;



Q_X = 1;
Q_Y = 1;
Q_THETA = 1;
Q_VEL_RIGHT = 1;
Q_VEL_LEFT = 1;
Q_I_RIGHT = 0;
Q_I_LEFT = 0;

R_V_RIGHT = 0.1;
R_V_LEFT = 0.1;
