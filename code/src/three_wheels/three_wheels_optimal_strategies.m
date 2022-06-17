function [optimal_u, optimal_d] = three_wheels_optimal_strategies(t, data, deriv, schemeData, x)
  checkStructureFields(schemeData, 'grid', 'd_sys');%'range_input_d', 'range_input_u', 'L', 'DELTA');

  grid = schemeData.grid;
  d_sys = schemeData.d_sys;

  d_min = d_sys.range_input_d(1); % schemeData.range_input_d(1);
  d_max = d_sys.range_input_d(2); %schemeData.range_input_d(2);

  u_min = d_sys.range_input_u(1); %schemeData.range_input_u(1);
  u_max = d_sys.range_input_u(2); %schemeData.range_input_u(2);

  p1 = deriv{1};
  p2 = deriv{2};
  p3 = deriv{3};

  L = d_sys.L; %schemeData.L;
  DELTA = d_sys.DELTA; %schemeData.DELTA;

  if nargin < 5
    x_r = grid.xs{1};
    y_r = grid.xs{2};
    theta = grid.xs{3};      
  else
    x_r = x(1);
    y_r = x(2);
    theta = x(3);  
  end

  %disp('size(x_r)');
  %disp(size(x_r));
  % -----------------------------------------------------------
  gamma_u_1 = (2/3)*cos(theta+DELTA) .* p1 ...
                + (2/3)*sin(theta+DELTA) .* p2 ...
                + (1/(3*L)) .* p3;

  gamma_u_2 = (-2/3)*cos(theta-DELTA) .* p1 ...
              + (-2/3)*sin(theta-DELTA) .* p2 ...
              + (1/(3*L)) .* p3;

  gamma_u_3 = (2/3)*sin(theta) .* p1 ...
          + (-2/3)*cos(theta) .* p2 ...
          + (1/(3*L)) .* p3;
  % -----------------------------------------------------------

  % -----------------------------------------------------------
  gamma_d = (-sin(theta)) .* p1 + cos(theta) .* p2;
  % -----------------------------------------------------------

  % -----------------------------------------------------------
  opt_input_u_1 = (gamma_u_1 >= 0)*u_min + (gamma_u_1 < 0)*u_max;
  opt_input_u_2 = (gamma_u_2 >= 0)*u_min + (gamma_u_2 < 0)*u_max;
  opt_input_u_3 = (gamma_u_3 >= 0)*u_min + (gamma_u_3 < 0)*u_max;

  optimal_u = {opt_input_u_1; opt_input_u_2; opt_input_u_3};
  % -----------------------------------------------------------

  % -----------------------------------------------------------
  opt_input_d_1 = (gamma_d >= 0)*d_max + (gamma_d < 0)*d_min;
  opt_input_d_2 = opt_input_d_1;
  opt_input_d_3 = 0;

  optimal_d = {opt_input_d_1; opt_input_d_2; opt_input_d_3};
  % -----------------------------------------------------------
%---------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------------------------------------------------------------
