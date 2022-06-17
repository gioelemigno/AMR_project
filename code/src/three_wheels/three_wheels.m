
%---------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------------------------------------------------------------
function [ data, g, data0 ] = three_wheels(accuracy, env_type, x0, disturbance_strategy)

  close all;
  % INCLUDE ---------------------------------------------------------------------------
    run('../include_libs');
  %---------------------------------------------------------------------------

  % INTEGRATOR PARAMETERS ---------------------------------------------------------------------------
    tMax = 10;%3;%2.8;                  % End time.
    plotSteps = 30;%9;               % How many intermediate plots to produce?
    t0 = 0;                      % Start time.
    singleStep = 0;              % Plot at each timestep (overrides tPlot).

    % Period at which intermediate plots should be produced.
    tPlot = (tMax - t0) / (plotSteps - 1);

    % How close (relative) do we need to get to tMax to be considered finished?
    small = 10 * eps;

    % What kind of dissipation?
    dissType = 'global';
  %---------------------------------------------------------------------------

      % Loop until tMax (subject to a little roundoff).
      tNow = t0;
                 
      dt = tPlot;
      tau = t0:dt:tMax;

  %targetRadius = 5;


  % PLOT PARAMETERS ---------------------------------------------------------------------------
    % What level set should we view?
    level = 0;

    % Visualize the 3D reachable set.
    displayType = 'surface';

    % Pause after each plot?
    pauseAfterPlot = 0;

    % Delete previous plot before showing next?
    deleteLastPlot = 1;

    % Visualize the angular dimension a little bigger.
    aspectRatio = [ 1 1 1]; %[ 1 1 0.4 ];

    % Plot in separate subplots (set deleteLastPlot = 0 in this case)?
    useSubplots = 0;
  %---------------------------------------------------------------------------
  
  % DISCRETE SPACE ---------------------------------------------------------------------------  
    % Approximately how many grid cells?
    %   (Slightly different grid cell counts will be chosen for each dimension.)
    Nx = 85;%51;%51;

    x_min = -17;
    x_max = 17;
  
    y_min = -17;
    y_max = 17;

    grid_min = [  x_min; y_min;     -pi]; % Lower corner of computation domain
    grid_max = [ x_max; y_max; +pi ];    % Upper corner of computation domain
  
    %  grid_max = [ +20; +10; +pi ];    % Upper corner of computation domain
    
    
    %grid_min = [  -11; -11;     -pi ]; % Lower corner of computation domain
    %grid_max = [ +11; +11; +pi ];    % Upper corner of computation domain
    % Create the grid.
    g.dim = 3;
    g.min = grid_min;
    g.max = grid_max;
    g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate; @addGhostPeriodic };
    % Roughly equal dx in x and y (so different N).
    g.N = [ Nx; Nx; Nx];
    %g.N = [ Nx; ceil(Nx * (g.max(2) - g.min(2)) / (g.max(1) - g.min(1))); Nx-1 ];
    % Need to trim max bound in \psi (since the BC are periodic in this dimension).
    g.max(3) = g.max(3) * (1 - 1 / g.N(3));
    g = processGrid(g);
  %---------------------------------------------------------------------------
  

  % DYNAMIC SYSTEM ----------------------------------------------------------------
    range_input_d = [-0.15, 0.15];
    range_input_u = [-1.5, 1.5];%[-3.5, 3.5];

    L = 1;
    DELTA = deg2rad(30);
    d_sys = Three_wheels_model(DELTA, L, range_input_u, range_input_d);
  % -------------------------------------------------------------------------------

  % SETUP SCHEMEDATA--------------------------------------------------------------------------
    % additional arguments:
    schemeData.d_sys = d_sys;
    
    % Set up spatial approximation scheme.
    schemeFunc = @termLaxFriedrichs;
    schemeData.hamFunc = @three_wheels_HamFunc;%@d_sys.in_class_three_wheels_HamFunc;% 
    schemeData.partialFunc = @three_wheels_PartialFunc; 
    schemeData.grid = g;

    % Choose degree of dissipation.
    switch(dissType)
      case 'global'
        schemeData.dissFunc = @artificialDissipationGLF;
      case 'local'
        schemeData.dissFunc = @artificialDissipationLLF;
      case 'locallocal'
        schemeData.dissFunc = @artificialDissipationLLLF;
      otherwise
        error('Unknown dissipation function %s', dissFunc);
    end

    %{
    if(nargin < 1)
      accuracy = 'medium';
    end
    %}

    % Set up time approximation scheme.
    integratorOptions = odeCFLset('factorCFL', 0.75, 'stats', 'on');

    % Choose approximations at appropriate level of accuracy.
    switch(accuracy)
      case 'low'
        schemeData.derivFunc = @upwindFirstFirst;
        integratorFunc = @odeCFL1;
      case 'medium'
        schemeData.derivFunc = @upwindFirstENO2;
        integratorFunc = @odeCFL2;
      case 'high'
        schemeData.derivFunc = @upwindFirstENO3;
        integratorFunc = @odeCFL3;
      case 'veryHigh'
        schemeData.derivFunc = @upwindFirstWENO5;
        integratorFunc = @odeCFL3;
      otherwise
        error('Unknown accuracy level %s', accuracy);
    end

    if(singleStep)
      integratorOptions = odeCFLset(integratorOptions, 'singleStep', 'on');
    end

    % Restrict the Hamiltonian so that reachable set only grows.
    %   The Lax-Friedrichs approximation scheme MUST already be completely set up.
    innerFunc = schemeFunc;
    innerData = schemeData;
    clear schemeFunc schemeData;

    
    % Wrap the true Hamiltonian inside the term approximation restriction routine.
    schemeFunc = @termRestrictUpdate;
    schemeData.innerFunc = innerFunc;
    schemeData.innerData = innerData;
    schemeData.positive = 0;
  %---------------------------------------------------------------------------

  % CREATE ENVIRONMENT ---------------------------------------------------------------------------
    Ref_x = 0;
    Ref_y = 6;

    velocity = -1;

    R = (4*L)/3;
    
    switch(env_type)
      case 'static'
        env = Three_wheels_static_environment(g, tau, Ref_x, Ref_y, L, velocity, R);
      case 'dynamic'
        env = Three_wheels_dynamic_environment(g, tau, Ref_x, Ref_y, L, velocity, R);
      otherwise
        error('Unknown env_type %s', env_type);
    end

    
    
  %---------------------------------------------------------------------------


  % MAIN LOOP ---------------------------------------------------------------------------
    %% datastructures setup ........................................
      clns = repmat({':'}, 1, g.dim);
      dim = g.N';

      V_x_t = zeros([dim(1:g.dim) length(tau)]);

      avoid_set_t_end = env.get_avoid_set_at(length(tau));
      constr_set_t_end = -avoid_set_t_end;
      reach_set_t_end = env.get_reach_set_at(length(tau));
      
      data0 = max(reach_set_t_end, constr_set_t_end);
      
      V_x_t(clns{:}, length(tau)) = data0;
    %% .............................................................

    %% setup plot ............................................
      f = figure;
      % Set up subplot parameters if necessary.
      if(useSubplots)
        rows = ceil(sqrt(plotSteps));
        cols = ceil(plotSteps / rows);
        plotNum = 1;
        subplot(rows, cols, plotNum);
      end

      h = visualizeLevelSet_color(g, data0, displayType, level, [ 't = ' num2str(tMax - tNow) ], 'blue');
      h = visualizeLevelSet_color(g, reach_set_t_end, displayType, level, [ 't = ' num2str(tMax - tNow) ], 'green');
      h = visualizeLevelSet_color(g, avoid_set_t_end, displayType, level, [ 't = ' num2str(tMax - tNow) ], 'red');

      camlight right;  camlight left; camlight headlight;
      hold on;
      axis(g.axis);
      daspect(aspectRatio);
      drawnow;
    %%..........................................................

    xlabel('$x_r$', 'interpreter', 'latex', "FontSize", 30);
    ylabel('$y_r$', 'interpreter', 'latex', "FontSize", 30);
    zlabel('$\theta$', 'interpreter', 'latex', "FontSize", 30);
    %set(gca, "FontSize", 20);
    disp("set manually the view and press Enter");
    pause;
    
    saveas(f, join(['V_comp/V_comp_it=', num2str(0), '_t=', num2str(tMax- tNow), '.png']));  

    i = 1;
    data = data0;
    startTime = cputime;
    while(tMax - tNow > small * tMax)
      %pause;
      % Get correct figure, and remember its current view.
      figure(f);
      [ view_az, view_el ] = view;
      clf;
      camlight right;  camlight left;
      hold on;
      axis(g.axis);
      daspect(aspectRatio);
      %drawnow;

      delete(h);
      % Reshape data array into column vector for ode solver call.
      y0 = data(:);

      % How far to step?
      tSpan = [ tNow, min(tMax, tNow + tPlot) ];
      
      % Take a timestep.
      [ t y ] = feval(integratorFunc, schemeFunc, tSpan, y0,...
                      integratorOptions, schemeData);
      tNow = t(end);

      % Get back the correctly shaped data array
      data = reshape(y, g.shape);

      reach_set_t_now = env.get_reach_set_at(length(tau)-i);
      data = min(data, reach_set_t_now);

      avoid_set_t_now = env.get_avoid_set_at(length(tau)-i);
      constr_set_t_now = -avoid_set_t_now;
      data = max(data, constr_set_t_now);

      if(pauseAfterPlot)
        % Wait for last plot to be digested.
        pause;
      end



      % Move to next subplot if necessary.
      if(useSubplots)
        plotNum = plotNum + 1;
        subplot(rows, cols, plotNum);
      end

      % Create new visualization.
      h = visualizeLevelSet_color(g, data, displayType, level, [ 't = ' num2str(tMax- tNow) ], 'blue');
      h = visualizeLevelSet_color(g, reach_set_t_now, displayType, level, [ 't = ' num2str(tMax-tNow) ], 'green');
      h = visualizeLevelSet_color(g, avoid_set_t_now, displayType, level, [ 't = ' num2str(tMax-tNow) ], 'red');
 
      xlabel('$x_r$', 'interpreter', 'latex', "FontSize", 30);
      ylabel('$y_r$', 'interpreter', 'latex', "FontSize", 30);
      zlabel('$\theta$', 'interpreter', 'latex', "FontSize", 30);
      %set(gca, "FontSize", 20);
      % Restore view.
      view(view_az, view_el);
      saveas(f, join(['V_comp/V_comp_it=', num2str(i, '%03.f'), '_t=', num2str(tMax- tNow), '.png']));  

      V_x_t(clns{:}, length(tau)-i) = data;
      i = i+1;
      %pause;
    end
  %---------------------------------------------------------------------------
  endTime = cputime;

  fprintf('Total execution time %g seconds\n', endTime - startTime);



  V_x_0 = V_x_t(clns{:}, 1);
  %V_x_T = V_x_t(clns{:}, length(tau));

  %static env
  %x0 = [-5, -8 -2];
  
  %dynamic_env
  %x0 = [0, -7, -3];
  %x0 = [0, 0, -3];
  %x0 = [14, 14, -3];
  

  
  if(~is_inside(g, V_x_0, x0))
    disp("The initial state x0 is not in the BRS at t=0");
    %err;
  end


  %[traj, traj_tau] = d_sys.three_wheels_optimal_trajectory(x0, g, V_x_t, tau, env);
  %disturbance_strategy = 'random_normal';%'optimal'; %'random_normal';
  [traj, traj_tau, u_t, d_t] = d_sys.three_wheels_optimal_trajectory(x0, g, V_x_t, tau, env, disturbance_strategy);


  disp(u_t);  
  disp(d_t);

  f_u_d = figure();
  for i=1:size(u_t, 2)
    clf;
    f_u_d.WindowState = 'maximized';

    t_t_now = tau(1:i);
    u_t_t_now = u_t(:, 1:i);
    d_t_t_now = d_t(:, 1:i);

    tiledlayout(3,2);

    nexttile
    plot(t_t_now, u_t_t_now(1, :), '-o', 'LineWidth', 3);
    title('$u_1(t)$', 'interpreter', 'latex', "FontSize", 30);
    set(gca, 'xtick', [tau(1):1:tau(end)]);
    set(gca, 'ytick', [range_input_u(1)-1:1:range_input_u(2)+1]);
    axis([tau(1) tau(end) range_input_u(1)-1 range_input_u(2)+1]);

    nexttile
    plot(t_t_now, d_t_t_now(1, :), '-o', 'LineWidth', 3);
    title('$d(t)$', 'interpreter', 'latex', "FontSize", 30);
    set(gca, 'xtick', [tau(1):1:tau(end)]);
    set(gca, 'ytick', [range_input_d(1)-0.15:0.15:range_input_d(2)+0.15]);
    axis([tau(1) tau(end) range_input_d(1)-0.15 range_input_d(2)+0.15]);
  
    nexttile
    plot(t_t_now, u_t_t_now(2, :), '-o', 'LineWidth', 3);
    title('$u_2(t)$', 'interpreter', 'latex', "FontSize", 30);
    set(gca, 'xtick', [tau(1):1:tau(end)]);
    set(gca, 'ytick', [range_input_u(1)-1:1:range_input_u(2)+1]);
    axis([tau(1) tau(end) range_input_u(1)-1 range_input_u(2)+1]);

    nexttile
    %plot(t_t_now, d_t_t_now(2, :), '-o', 'LineWidth', 3);
    %title('$d_2(t)$', 'interpreter', 'latex', "FontSize", 30);
    %set(gca, 'xtick', [tau(1):1:tau(end)]);
    %set(gca, 'ytick', [range_input_d(1)-0.15:0.15:range_input_d(2)+0.15]);
    %axis([tau(1) tau(end) range_input_d(1)-0.15 range_input_d(2)+0.15]);

    nexttile
    plot(t_t_now, u_t_t_now(3, :), '-o', 'LineWidth', 3);
    title('$u_3(t)$', 'interpreter', 'latex', "FontSize", 30);
    set(gca, 'xtick', [tau(1):1:tau(end)]);
    set(gca, 'ytick', [range_input_u(1)-1:1:range_input_u(2)+1]);
    axis([tau(1) tau(end) range_input_u(1)-1 range_input_u(2)+1]);

    pause(0.2);
    saveas(f_u_d, join(['inputs/', 'it=', num2str(i, '%03.f'), '_t=', num2str(tau(i)), '.png']));  

  end



  
  disp(u_t);



  x_r_t = traj(1, :);
  y_r_t = traj(2, :);
  psi_t = traj(3, :);


  fff = figure();%'visible', 'off');


  plot_game_C_space_2D(tau, g, env, V_x_t, fff, x_r_t, y_r_t);
  

  ff = figure;

  for i=1:length(x_r_t)
    %saveas(ff, join(['traj/', 'aa_del.png']));  
    
    clf;
    ff.WindowState = 'maximized';

    %hold off;
    %hold on;
  %  clf;
    color_alpha = 0.5;
    
    plot(x_r_t(1:i), y_r_t(1:i), 'k', 'LineWidth', 2);
    hold on;
    env.plot_env_at(i);
    
    %hold on;

    plot_three_wheel(x_r_t(i), y_r_t(i), psi_t(i), L, 'magenta', color_alpha);
    title(['t = ' num2str(tau(i))]);

    xlabel('$x_w$', 'interpreter', 'latex', "FontSize", 30);
    ylabel('$y_w$', 'interpreter', 'latex', "FontSize", 30);
    %axis manual;
    
    %axis(g.axis);
    %axis equal;
%    set(gcf, 'Position', get(0, 'Screensize'));

    grid on;
    set(gca, 'DataAspectRatio', [1 1 1]);
    set(gca, 'xtick', [x_min:2:x_max]);
    set(gca, 'ytick', [y_min:2:y_max]);
    axis([x_min x_max y_min y_max]);
    %pause;
    pause(0.2);
    saveas(ff, join(['traj/', 'traj_it=', num2str(i, '%03.f'), '_t=', num2str(tau(i)), '.png']));  
    
  end

err;









