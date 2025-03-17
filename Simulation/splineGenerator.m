function [xFine, yFine, R] = splineGenerator(x, y)
    t = 1:length(x);

    t_fine = linspace(min(t), max(t), 1000);
    
    xFine = spline(t, x, t_fine);
    yFine = spline(t, y, t_fine);
    
    dx_dt = gradient(xFine, t_fine);
    dy_dt = gradient(yFine, t_fine);
    
    d2x_dt2 = gradient(dx_dt, t_fine);
    d2y_dt2 = gradient(dy_dt, t_fine);
    
    numerator = (dx_dt.^2 + dy_dt.^2).^(3/2);
    denominator = abs(dx_dt .* d2y_dt2 - dy_dt .* d2x_dt2);
    
    R = numerator ./ (denominator + eps);
end