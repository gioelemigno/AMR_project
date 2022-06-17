
function alpha = three_wheels_PartialFunc(t, data, derivMin, derivMax, schemeData, dim)

  checkStructureFields(schemeData, 'grid', 'd_sys');%'range_input_d', 'range_input_u', 'L', 'DELTA');

  grid = schemeData.grid;
  d_sys = schemeData.d_sys;

  opt_input_u = evalin('base', 'opt_input_u');
  opt_input_d = evalin('base', 'opt_input_d');

  L = d_sys.L;%schemeData.L;
  DELTA = d_sys.DELTA;%schemeData.DELTA;

  x_r = grid.xs{1};
  y_r = grid.xs{2};
  theta = grid.xs{3};



  opt_input_u_1 = opt_input_u{1}; 
  opt_input_u_2 = opt_input_u{2}; 
  opt_input_u_3 = opt_input_u{3};

  
  opt_input_d_1 = opt_input_d{1};
  opt_input_d_2 = opt_input_d{2};
  opt_input_d_3 = opt_input_d{3};

  switch dim
    case 1
      alpha = (2/3) .* cos(theta+DELTA) .* opt_input_u_1 ...
            + (-2/3) .* cos(theta-DELTA) .* opt_input_u_2 ...
            + (2/3) .* sin(theta) .* opt_input_u_3 ...
            + (-sin(theta)) .* opt_input_d_1;
      
      alpha = abs(alpha); 


    case 2
      alpha = (2/3) .* sin(theta+DELTA) .* opt_input_u_1 ...
            + (-2/3) .* sin(theta-DELTA) .* opt_input_u_2 ...
            + (-2/3) .* cos(theta) .* opt_input_u_3 ...
            + cos(theta) .* opt_input_d_2;
      
      alpha = abs(alpha);


    case 3
      alpha = (opt_input_u_1 + opt_input_u_2 + opt_input_u_3) .* (1/(3*L));
      alpha = abs(alpha);

    otherwise
      error([ 'Partials' ...
              ' only exist in dimensions 1-3' ]);
  end
