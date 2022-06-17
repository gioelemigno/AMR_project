
function hamValue = three_wheels_HamFunc(t, data, deriv, schemeData)
  checkStructureFields(schemeData, 'grid', 'd_sys');%, 'range_input_d', 'range_input_u', 'L', 'DELTA');

  grid = schemeData.grid;
  d_sys = schemeData.d_sys;
  
  %global opt_input_d;
  %global opt_input_u;

  p1 = deriv{1};
  p2 = deriv{2};
  p3 = deriv{3};

  L = d_sys.L;%schemeData.L;
  DELTA = d_sys.DELTA;%schemeData.DELTA;

  x_r = grid.xs{1};
  y_r = grid.xs{2};
  theta = grid.xs{3};

  

  [opt_input_u, opt_input_d] = three_wheels_optimal_strategies(t, data, deriv, schemeData);
  %[opt_input_u, opt_input_d] = d_sys.three_wheels_optimal_strategies(t, data, deriv, schemeData);
  assignin('base', 'opt_input_u', opt_input_u);
  assignin('base', 'opt_input_d', opt_input_d);

  opt_input_u_1 = opt_input_u{1}; 
  opt_input_u_2 = opt_input_u{2}; 
  opt_input_u_3 = opt_input_u{3};

  opt_input_d_1 = opt_input_d{1};
  opt_input_d_2 = opt_input_d{2};
  opt_input_d_3 = opt_input_d{3};

  ham_u1 = (2/3) .* cos(theta+DELTA) .* p1 + (2/3) .* sin(theta + DELTA) .* p2 + (1/(3*L)) .* p3;
  ham_u2 = (-2/3) .* cos(theta-DELTA) .* p1 + (-2/3) .* sin(theta-DELTA) .* p2 + (1/(3*L)) .* p3;
  ham_u3 = (2/3) .* sin(theta) .* p1 + (-2/3) .* cos(theta) .* p2 + (1/(3*L)) .* p3;

  ham_d1 = -sin(theta) .* p1;
  ham_d2 = cos(theta) .* p2;
  ham_d3 = 0;

  hamValue = ham_u1 .* opt_input_u_1 ...
            + ham_u2 .* opt_input_u_2 ...
            + ham_u3 .* opt_input_u_3 ...
            + ham_d1 .* opt_input_d_1 ...
            + ham_d2 .* opt_input_d_2 ...
            + ham_d3 .* opt_input_d_3;

  hamValue = (-1)*hamValue;
