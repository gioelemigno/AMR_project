function plot_rectangle(lower_point, upper_point, rotation, color, color_alpha)
    if nargin < 6
        color_alpha = 0.5;
    end

    theta = rad2deg(rotation);
    L_x = upper_point(1)-lower_point(1);
    L_y = upper_point(2)-lower_point(2);

    x_a = lower_point(1);
    y_a = lower_point(2);

    x_b = x_a;
    y_b = y_a + L_y;

    x_c = upper_point(1);
    y_c = upper_point(2);

    x_d = x_c;
    y_d = y_c - L_y;

    hold on;

    
    middle_x = lower_point(1) + L_x/2;
    middle_y = lower_point(2) + L_y/2;

    x = [x_a x_b x_c x_d];
    y = [y_a y_b y_c y_d];



    ps = polyshape(x, y);
    ps = rotate(ps, theta, [middle_x middle_y]);
    pl = plot(ps);
    pl.FaceColor = color;
    pl.FaceAlpha = color_alpha;
    hold off;
