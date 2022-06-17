classdef Three_wheels_model
  properties
    DELTA %costant 30 degree (wheel orientation in the Robot coordinate system)
    L %length (distance from center)
    x 

    range_input_d
    range_input_u
  end
  
  methods
    function obj = Three_wheels_model(delta, L, range_input_u, range_input_d)
        obj.x = 0;
        obj.DELTA = delta;
        obj.L = L;

        obj.range_input_u = range_input_u;
        obj.range_input_d = range_input_d;
    end
    
    function dx = three_wheels_dynamics(obj, t, x, u, d)
        dx = zeros(3, 1);

        L = obj.L;
        delta = obj.DELTA;

        theta = x(3);

        dx(1) = u(1) * 2/3 * cos(theta + delta) - u(2) * 2/3 * cos(theta-delta) + u(3) * 2/3 * sin(theta) - sin(theta) * d(1);
        dx(2) = u(1) * 2/3 * sin(theta + delta) - u(2)* 2/3 * sin(theta-delta) - u(3) * 2/3 * cos(theta) + cos(theta) * d(2);
        dx(3) = u(1) * 1/(3*L) + u(2)* 1/(3*L) + u(3) * 1/(3*L) ;
    end

    function x1 = three_wheels_update_state(obj, x0, u, d, T)
        if T == 0
            x1 = x0;
            return
        end
        
        [~, x] = ode113(@(t,x) obj.three_wheels_dynamics(t, x, u, d), [0 T], x0);
    
        x1 = x(end, :)';
        obj.x = x1;
    end



    function [traj, traj_tau, u_t, d_t] = three_wheels_optimal_trajectory(obj, x0, g, V_x_t, tau, environment, disturb_strategy)
      %% FUNCTION INSPIRED BY computeOptTraj() from helperOC 

      if any(diff(tau)) < 0
        error('Time stamps must be in ascending order!')
      end

      rng('default');
      max_d_abs = min(abs(obj.range_input_d(1)), abs(obj.range_input_d(2)));

      mu = 0;
      sigma = max_d_abs/3;
      
      min_clip_value = obj.range_input_d(1);
      max_clip_value =  obj.range_input_d(2);

      if strcmp(disturb_strategy, 'optimal')
        disp('optimal disturbance');
      elseif strcmp(disturb_strategy, 'random_normal')
        disp('random_normal disturbance')
        fprintf('Random Normal Guassinan noise, mu=%4.2f, sigma=%4.2f', mu, sigma);
      else
        disp("Unknown disturb strategy");
        traj = -1;
        traj_tau = -1;
        return;
      end

      clns = repmat({':'}, 1, g.dim);

      subSamples = 10;%5;
      tauLength = length(tau);
      dtSmall = (tau(2) - tau(1))/subSamples;

      obj.x = x0;
      % Initialize trajectory
      traj = nan(g.dim, tauLength);
      traj(:,1) = obj.x;

      u_t = nan(3, tauLength-1);
      d_t = nan(3, tauLength-1);

      iter = 1;
      while iter <= tauLength 
        % V_x_t at current time
        V_x_now = V_x_t(clns{:}, iter);
        
        % Update trajectory
        Deriv = computeGradients(g, V_x_now);
        for j = 1:subSamples
          deriv_array = eval_u(g, Deriv, obj.x);
          deriv = {deriv_array(1), deriv_array(2), deriv_array(3)};
 
          schemeData.grid = g;
          schemeData.d_sys = obj;

          [optimal_u, optimal_d] = three_wheels_optimal_strategies(tau(iter), V_x_now, deriv, schemeData, obj.x);
          
          u = cell2mat(optimal_u);
          d = cell2mat(optimal_d);
          
          if strcmp(disturb_strategy, 'random_normal')
            d = randn(1)*sigma + mu;
            d = ones(size(optimal_d))*d;
            %clipping
            if d < min_clip_value
              d = min_clip_value;
            elseif d > max_clip_value
              d = max_clip_value;
            end
            d(3)= 0; %only position disturbance
            %disp(d);
          end

          obj.x = obj.three_wheels_update_state(obj.x, u, d, dtSmall);
        end

        reach_set = environment.get_reach_set_at(iter);
        if is_inside(g, reach_set, obj.x)
          string = sprintf("reach_set reachable at t=%.2f", tau(iter));
          disp(string);
          %break;
        end

        u_t(:,iter) = u;
        d_t(:,iter) = d;

        % Record new point on nominal trajectory
        iter = iter + 1;
        traj(:,iter) = obj.x;
      end

      % Delete unused indices
      traj(:,iter:end) = [];
      traj_tau = tau(1:iter-1);
     
      u_t(:,iter-1:end) = [];
      d_t(:,iter-1:end) = [];
    end


  end % end methods
end % end classdef

