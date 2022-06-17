function plot_game_C_space_2D(tau, g, env, V_x_t, fff, x_r_t, y_r_t)
    x_min = -17;
    x_max = 17;
  
    y_min = -17;
    y_max = 17;

    clns = repmat({':'}, 1, g.dim);
    
    for k=1:length(tau)
        clf;

        fff.WindowState = 'maximized';

        [g2D, data2D_reach_set] = proj(g, env.get_reach_set_at(k), [0 0 1]);
        [g2D, data2D_avoid_set] = proj(g, env.get_avoid_set_at(k), [0 0 1]);
        [g2D, data2D_V_x_t] = proj(g, V_x_t(clns{:}, k), [0 0 1]);


        % REACH-SET
        [c, h] = contourf(g2D.xs{1}, g2D.xs{2}, data2D_reach_set, [-9999999 0]);%, 'color', 'red');
        cmap = [0 1 0; 1 1 1];
        colormap(cmap);
        set(gca, 'DataAspectRatio', [1 1 1]);
        set(gca, 'xtick', [x_min:2:x_max]);
        set(gca, 'ytick', [y_min:2:y_max]);
        axis([x_min x_max y_min y_max]);
        pause(1);
        rs_img = getframe(gca).cdata;
        clf;

        % AVOID-SET
        [c, h] = contourf(g2D.xs{1}, g2D.xs{2}, data2D_avoid_set, [-9999999 0]);%, 'color', 'red');
        cmap = [1 0 0; 1 1 1];
        colormap(cmap);
        set(gca, 'DataAspectRatio', [1 1 1]);
        set(gca, 'xtick', [x_min:2:x_max]);
        set(gca, 'ytick', [y_min:2:y_max]);
        axis([x_min x_max y_min y_max]);
        as_img = getframe(gca).cdata;
        clf;

        % V_x_t
        [c, h] = contourf(g2D.xs{1}, g2D.xs{2}, data2D_V_x_t, [-9999999 0]);%, 'color', 'red');
        cmap = [0 0 1; 1 1 1];
        colormap(cmap);
        set(gca, 'DataAspectRatio', [1 1 1]);
        set(gca, 'xtick', [x_min:2:x_max]);
        set(gca, 'ytick', [y_min:2:y_max]);
        axis([x_min x_max y_min y_max]);
        v_x_t_img = getframe(gca).cdata; %print('-RGBImage');
        clf;

        % TRAJECTORY
        [c, h] = contourf(g2D.xs{1}, g2D.xs{2}, data2D_reach_set, [-9999999 0]);%, 'color', 'red');
        cmap = [0 1 0; 1 1 1];
        colormap(cmap);
        set(gca, 'DataAspectRatio', [1 1 1]);
        set(gca, 'xtick', [x_min:2:x_max]);
        set(gca, 'ytick', [y_min:2:y_max]);
        axis([x_min x_max y_min y_max]);
        hold on;
        if k == length(tau)
            plot(x_r_t(k), y_r_t(k), 'k.', 'MarkerSize', 50, 'LineWidth', 4);
        end
        if k == 1
            plot(x_r_t(k), y_r_t(k), 'm+', 'MarkerSize', 30, 'LineWidth', 4);
        else
            plot(x_r_t(1:k), y_r_t(1:k), 'LineWidth', 4, 'color', 'magenta');
        end
        hold off;
        plot_img = getframe(gca).cdata;
        clf;

        % put everthing together
        res_img = ones(size(rs_img)) .* 100;
        background = [255, 255, 255];
        for i=1:size(res_img, 1)
            for j=1:size(res_img, 2)
                rs = [rs_img(i,j,1), rs_img(i,j,2), rs_img(i,j,3)];
                as = [as_img(i,j,1), as_img(i,j,2), as_img(i,j,3)];
                v_x_t = [v_x_t_img(i,j,1), v_x_t_img(i,j,2), v_x_t_img(i,j,3)];

                plot_p = [plot_img(i,j,1), plot_img(i,j,2), plot_img(i,j,3)]; 
                
                if ~isequal(as, background)
                    res_img(i,j,:) = as;
                end
                if ~isequal(v_x_t, background)
                    res_img(i,j,:) = v_x_t;
                end
                if ~isequal(rs, background)
                    res_img(i,j,:) = rs;
                end
                if ~isequal(plot_p, background)
                    res_img(i,j,:) = plot_p;
                end
            end
        end
  

        fff.WindowState = 'maximized';
  
        imshow(res_img);
        fff.WindowState = 'maximized';
        pause(1);
        saveas(fff, join(['v_2d/game_C_space_k=', num2str(k, '%03.f'), '_t=', num2str(tau(k)), '_.png']));  
    end
end
