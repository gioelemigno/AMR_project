classdef Three_wheels_static_environment
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

    lower_4
    upper_4
    
    L
    velocity

    R

    reach_set
    avoid_set
  end
  
  methods
    function obj = Three_wheels_static_environment(grid, tau, ref_x, ref_y, L, velocity, R)
        obj.tau = tau; 
        obj.grid = grid;
        
        obj.ref_x = ref_x;
        obj_ref_y = ref_y;

        obj.L = L;
        obj.velocity = velocity;

        obj.R = R;
      
        % Reach set
        obj.lower_rs = [-2*L, -2*L, -0.1];%-pi/4-0.05];%-0.5];%grid_min(3)];
        obj.upper_rs = [2*L, 2*L, 0.1];%-pi/4+0.05];%0.5];%grid_max(3)];

        % Avoid set
        epsilon = 1;
        L = obj.L;

        low_theta = obj.grid.min(3) - epsilon;
        upper_theta = obj.grid.max(3) + epsilon;

        obj.low_theta = low_theta;
        obj.upper_theta = upper_theta;

        obj.lower_1 = [-5*L, -5*L, low_theta];
        obj.upper_1 = [-3*L, -3*L, upper_theta];

        obj.lower_2 = [-5*L, 1*L, low_theta];
        obj.upper_2 = [-3*L, 6*L, upper_theta];
       
        obj.lower_3 = [0*L, 3*L, low_theta];
        obj.upper_3 = [2*L, 6*L, upper_theta];
        
        obj.lower_4 = [2*L, -5*L, low_theta];
        obj.upper_4 = [4*L, -3*L, upper_theta];

        obj.reach_set = obj.build_reach_set();
        obj.avoid_set = obj.build_avoid_set();
    end

    function reach_set = build_reach_set(obj)
      [c_lower, c_upper] = to_C_space(obj.lower_rs(1:2), obj.upper_rs(1:2), obj.R, 'reach_set');
      c_lower = [c_lower, obj.lower_rs(3)];
      c_upper = [c_upper, obj.upper_rs(3)];
      reach_set = shapeRectangleByCorners(obj.grid, c_lower, c_upper);

      obj.reach_set = reach_set;
    end


    function avoid_set = build_avoid_set(obj)
      low_theta = obj.low_theta;
      upper_theta = obj.upper_theta;

      [c_lower, c_upper] = to_C_space(obj.lower_1(1:2), obj.upper_1(1:2), obj.R, 'avoid_set');
      c_lower = [c_lower, low_theta];
      c_upper = [c_upper, upper_theta];
      obs_1 = shapeRectangleByCorners(obj.grid, c_lower, c_upper);

      [c_lower, c_upper] = to_C_space(obj.lower_2(1:2), obj.upper_2(1:2), obj.R, 'avoid_set');
      c_lower = [c_lower, low_theta];
      c_upper = [c_upper, upper_theta];
      obs_2 = shapeRectangleByCorners(obj.grid, c_lower, c_upper);

      [c_lower, c_upper] = to_C_space(obj.lower_3(1:2), obj.upper_3(1:2), obj.R, 'avoid_set');
      c_lower = [c_lower, low_theta];
      c_upper = [c_upper, upper_theta];
      obs_3 = shapeRectangleByCorners(obj.grid, c_lower, c_upper);

      [c_lower, c_upper] = to_C_space(obj.lower_4(1:2), obj.upper_4(1:2), obj.R, 'avoid_set');
      c_lower = [c_lower, low_theta];
      c_upper = [c_upper, upper_theta];
      obs_4 = shapeRectangleByCorners(obj.grid, c_lower, c_upper);

      avoid_set = obs_1;
      avoid_set = shapeUnion(avoid_set, obs_2);
      avoid_set = shapeUnion(avoid_set, obs_3);
      avoid_set = shapeUnion(avoid_set, obs_4);

      obj.avoid_set = avoid_set;
    end

    function reach_set = get_reach_set_at(obj, time_index)
      reach_set = obj.reach_set;
    end

    function avoid_set = get_avoid_set_at(obj, time_index)
      avoid_set = obj.avoid_set;
    end



    function plot_env_at(obj, time_index)
      tau = obj.tau;
      velocity = obj.velocity;

      t = tau(time_index);
      color_alpha = 0.5;

      plot_rectangle(obj.lower_rs(1:2), obj.upper_rs(1:2), 0, 'green', color_alpha);


      plot_rectangle(obj.lower_1(1:2), obj.upper_1(1:2), 0, 'red', color_alpha);
      plot_rectangle(obj.lower_2(1:2), obj.upper_2(1:2), 0, 'red', color_alpha);
      plot_rectangle(obj.lower_3(1:2), obj.upper_3(1:2), 0, 'red', color_alpha);
      plot_rectangle(obj.lower_4(1:2), obj.upper_4(1:2), 0, 'red', color_alpha);
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

  end % end methods
end % end classdef



  

