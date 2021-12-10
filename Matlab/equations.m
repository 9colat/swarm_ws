syms w_r w_l r l V v_x v_y theta w
eqn_v_x = v_x == (w_r*r + w_l*r)*cos(theta);
eqn_v_y = v_y == (w_r*r + w_l*r)*sin(theta);
eqn_w = w == r*(w_r-w_l)*l/2;

eqns = [eqn_v_x eqn_v_y eqn_w];
vars = [w_r w_l theta];

[sol_w_r sol_w_l sol_theta] = solve(eqns,vars)