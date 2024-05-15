clear all;

constants;

state = zeros(7, 1);
state(X) = 5;
state(THETA) = pi/2;
close all;



T = 20;
T_sim = 20;

Trials = 10;
fsf_errors = zeros(Trials, 1);
csc_errors = zeros(Trials, 1);

for trial = 1:Trials
    phi = @(t) 2*pi*t/(4*T) * (t/(t+0.5));

    
    conts = {};
    conts.J = normrnd(J0, 0.1*J0);
    conts.b = normrnd(b0, 0.1*b0);
    conts.K = normrnd(K0, 0.1*K0);
    conts.R = normrnd(R0, 0.1*R0);
    conts.L = normrnd(L0, 0.1*L0);

    conts.robot_mass = normrnd(robot_mass0, 0.1*robot_mass0);
    conts.robot_MoI = normrnd(robot_MoI0, 0.1*robot_MoI0);
    conts.robot_radius = normrnd(robot_radius0, 0.05*robot_radius0);
    conts.r_wheel = normrnd(r_wheel0, 0.05*r_wheel0);

    M = normrnd(0, 0.05);
    A = normrnd(0, 0.01);
    rad = @(t) 5+M; %5 + M*t + A*sin(2*pi*t/T);

    x_traj = @(t) rad(t) .* cos(phi(t));
    y_traj = @(t) rad(t) .* sin(phi(t));
    
    h = 0.00001;
    x_dot_traj = @(t) 1/h * (x_traj(t + h) - x_traj(t));
    y_dot_traj = @(t) 1/h * (y_traj(t + h) - y_traj(t));
    
    theta_traj = @(t) phi(t) + pi/2;%atan2(y_dot_traj(t), x_dot_traj(t));
    
    theta_dot_traj = @(t) 1/h * (theta_traj(t + h) - theta_traj(t));
    
    vels_traj = @(t) vels_from_derivs(conts, x_dot_traj(t), y_dot_traj(t), theta_dot_traj(t));
    vels_dot_traj = @(t) (vels_traj(t + h) - vels_traj(t)) / h;

    Is_traj = @(t) currents_from_vels(conts, vels_traj(t), vels_dot_traj(t));
    Is_dot_traj = @(t) 1/h * (Is_traj(t + h) - Is_traj(t));
    
    Vs_traj = @(t) voltage_from_currents(conts, vels_traj(t), Is_traj(t), Is_dot_traj(t));
    
    desired_state = @(t) [x_traj(t); y_traj(t); theta_traj(t); vels_traj(t); Is_traj(t)];


    Q_low = diag([Q_VEL_RIGHT, Q_VEL_LEFT, Q_I_RIGHT, Q_I_LEFT]);
    R_low = diag([R_V_RIGHT, R_V_LEFT]);

    [A_low, B_low] = motor_deriv_matrix(conts);

    [~, K_low, ~] = icare(A_low, B_low, Q_low, R_low);
    cascaded = @(t, x) linearized_cascade(conts, t, x, desired_state(t), Vs_traj(t), K_low);

    fsf_ctrl = @(t, x) full_state_feedback(conts, t, x, desired_state(t), Vs_traj(t));
    
    deriv_csc = @(t, x) model_deriv(conts, x, cascaded(t, x));
    deriv_fsf = @(t, x) model_deriv(conts, x, fsf_ctrl(t, x));
    
    %state = desired_state(0);
    
    [ts_csc, states_csc] = prop_difeq(deriv_csc, [0; T_sim], state);
    [ts_fsf, states_fsf] = prop_difeq(deriv_fsf, [0; T_sim], state); 

    [~, M] = size(ts_csc);

    csc_errors(trial) = sqrt(1/M * sum((states_csc(:, X) - x_traj(ts_csc)').^2 + (states_csc(:, Y) - y_traj(ts_csc)').^2));
    fsf_errors(trial) = sqrt(1/M * sum((states_fsf(:, X) - x_traj(ts_fsf)').^2 + (states_fsf(:, Y) - y_traj(ts_fsf)').^2));
    %csc_errors = zeros(Trials, 1);
end

figure(1)
plot(states_csc(:, X), states_csc(:, Y), "b")
hold on;
plot(states_fsf(:, X), states_fsf(:, Y), "r")

ts = ts_csc;

x_d = x_traj(ts);
y_d = y_traj(ts);
plot(x_d, y_d, "k-")
legend(["CSC", "FSF", "Nom"]);