classdef Three_wheels_dynamic_environment
  properties
    tau
    grid

    ref_x
    ref_y


    low_theta
    upper_theta

    lower_rs
    upper_rs


    lower_1
    upper_1

    lower_2
    upper_2

    lower_3
    upper_3
    
    L
    velocity

    R

    reach_set_t
    avoid_set_t
  end
  
  methods
    function obj = Three_wheels_dynamic_environment(grid, tau, ref_x, ref_y, L, velocity, R)
        obj.tau = tau; 
        obj.grid = grid;
        
        obj.ref_x = ref_x;
        obj_ref_y = ref_y;

        obj.L = L;
        obj.velocity = velocity;

        obj.R = R;
      
        % Reach set
        obj.lower_rs = [-2*L+ref_x, -2*L+ref_y, -0.1];
        obj.upper_rs = [2*L+ref_x, 2*L+ref_y, 0.1];
%        obj.lower_rs = [-2*L+ref_x, -2*L+ref_y, -0.5];
%        obj.upper_rs = [2*L+ref_x, 2*L+ref_y, 0.5];

        % Avoid set
        epsilon = 1;
        L = obj.L;

        low_theta = obj.grid.min(3) - epsilon;
        upper_theta = obj.grid.max(3) + epsilon;

        obj.low_theta = low_theta;
        obj.upper_theta = upper_theta;

        obj.lower_1 = [-2.5*L+ref_x, -2*L+ref_y, low_theta];
        obj.upper_1 = [-2*L+ref_x, 2*L+ref_y, upper_theta];
  
        obj.lower_2 = [-2*L+ref_x, 2*L+ref_y, low_theta];
        obj.upper_2 = [2*L+ref_x, 2.5*L+ref_y, upper_theta];
  
        obj.lower_3 = [-2*L+ref_x, -2.5*L+ref_y, low_theta];
        obj.upper_3 = [2*L+ref_x, -2*L+ref_y, upper_theta];

        obj.reach_set_t = obj.build_reach_set_t();
        obj.avoid_set_t = obj.build_avoid_set_t();
    end

    function reach_set_t = build_reach_set_t(obj)
      gDim = obj.grid.dim;
      clns = repmat({':'}, 1, gDim);
  
      dim = obj.grid.N';
      reach_set_t = zeros([dim(1:gDim) length(obj.tau)]);
      for j=1:length(obj.tau)
        t = obj.tau(j);
        [lower_rs_t, upper_rs_t] = obj.time_varying_y(obj.lower_rs, obj.upper_rs, obj.velocity, t);

        [c_lower, c_upper] = to_C_space(lower_rs_t(1:2), upper_rs_t(1:2), obj.R, 'reach_set');
        c_lower = [c_lower, obj.lower_rs(3)];
        c_upper = [c_upper, obj.upper_rs(3)];
        reach_set = shapeRectangleByCorners(obj.grid, c_lower, c_upper);
  
        reach_set_t(clns{:}, j) = reach_set;
      end
    end


    function avoid_set_t = build_avoid_set_t(obj)
      low_theta = obj.low_theta;
      upper_theta = obj.upper_theta;

      gDim = obj.grid.dim;
      clns = repmat({':'}, 1, gDim);
  
      dim = obj.grid.N';

      avoid_set_t = zeros([dim(1:gDim) length(obj.tau)]);
      for j=1:length(obj.tau)
        t = obj.tau(j);
  
        [lower_1_t, upper_1_t] = obj.time_varying_y(obj.lower_1, obj.upper_1, obj.velocity, t);
        [c_lower, c_upper] = to_C_space(lower_1_t(1:2), upper_1_t(1:2), obj.R, 'avoid_set');
        c_lower = [c_lower, low_theta];
        c_upper = [c_upper, upper_theta];
        obs_1 = shapeRectangleByCorners(obj.grid, c_lower, c_upper);
  
        [lower_2_t, upper_2_t] = obj.time_varying_y(obj.lower_2, obj.upper_2, obj.velocity, t);
        [c_lower, c_upper] = to_C_space(lower_2_t(1:2), upper_2_t(1:2), obj.R, 'avoid_set');
        c_lower = [c_lower, low_theta];
        c_upper = [c_upper, upper_theta];
        obs_2 = shapeRectangleByCorners(obj.grid, c_lower, c_upper);
  
        [lower_3_t, upper_3_t] = obj.time_varying_y(obj.lower_3, obj.upper_3, obj.velocity, t);
        [c_lower, c_upper] = to_C_space(lower_3_t(1:2), upper_3_t(1:2), obj.R, 'avoid_set');
        c_lower = [c_lower, low_theta];
        c_upper = [c_upper, upper_theta];
        obs_3 = shapeRectangleByCorners(obj.grid, c_lower, c_upper);
  
  
        avoid_set = obs_1;
        avoid_set = shapeUnion(avoid_set, obs_2);
        avoid_set = shapeUnion(avoid_set, obs_3);
  
        avoid_set_t(clns{:}, j) = avoid_set;
      end
    end

    function reach_set = get_reach_set_at(obj, time_index)
      gDim = obj.grid.dim;
      clns = repmat({':'}, 1, gDim);
      
      reach_set = obj.reach_set_t(clns{:}, time_index);
    end

    function avoid_set = get_avoid_set_at(obj, time_index)
      gDim = obj.grid.dim;
      clns = repmat({':'}, 1, gDim);

      avoid_set = obj.avoid_set_t(clns{:}, time_index);
    end

    function [lower_t, upper_t] = time_varying_y(obj, lower_0, upper_0, velocity, t)
      lxt = lower_0(1);
      lyt = lower_0(2) + velocity*t;
      lower_t = [lxt, lyt];
    
      uxt = upper_0(1);
      uyt = upper_0(2) + velocity*t;
      upper_t = [uxt, uyt];
    end


    function plot_env_at(obj, time_index)
      tau = obj.tau;
      velocity = obj.velocity;

      t = tau(time_index);
      color_alpha = 0.5;

      [lower_rs_t, upper_rs_t] = obj.time_varying_y(obj.lower_rs, obj.upper_rs, velocity, t);
      plot_rectangle(lower_rs_t(1:2), upper_rs_t(1:2), 0, 'green', color_alpha);


      [lower_1_t, upper_1_t] = obj.time_varying_y(obj.lower_1, obj.upper_1, velocity, t);
      plot_rectangle(lower_1_t(1:2), upper_1_t(1:2), 0, 'red', color_alpha);

      [lower_2_t, upper_2_t] = obj.time_varying_y(obj.lower_2, obj.upper_2, velocity, t);
      plot_rectangle(lower_2_t(1:2), upper_2_t(1:2), 0, 'red', color_alpha);

      [lower_3_t, upper_3_t] = obj.time_varying_y(obj.lower_3, obj.upper_3, velocity, t);
      plot_rectangle(lower_3_t(1:2), upper_3_t(1:2), 0, 'red', color_alpha);
    end


    function plot_env_level_set_at(obj, time_index)
      reach_set = obj.get_reach_set_at(time_index);
      avoid_set = obj.get_avoid_set_at(time_index);

      level = 0;
      displayType = 'surface';
      t = tau(time_index);

      h = visualizeLevelSet_color(obj.grid, reach_set, displayType, level, [ 't = ' num2str(t)], 'green');
      h = visualizeLevelSet_color(obj.grid, avoid_set, displayType, level, [ 't = ' num2str(t)], 'red');
    end

    function reach_set_t = get_reach_set_t(obj)
      reach_set_t = obj.reach_set_t;
    end
  end % end methods
end % end classdef

