function line = lsqLine(edges)
    % line = LSQLINE(edges) extract a line described in (alpha,r)
    % parameters from a set of points
    if (isempty(edges))
        line = [0;0];
    else
        x = edges(1,:);
        y = edges(2,:);

        n = length(x);
        Ex = sum(x);
        Ey = sum(y);
        Exy = x*y';
        Exx = x*x';
        Eyy = y*y';
        x_ = (1/n)*Ex;
        y_ = (1/n)*Ey;
        
        alpha = atan2((2*Ex*Ey-2*n*Exy),(Ex^2-Ey^2-n*Exx+n*Eyy))/2;
        r = x_ * cos(alpha) + y_ * sin(alpha);
        
        if (r < 0)
            r = -r;
            if (alpha < 0)
                alpha = alpha + pi;
            else
                alpha = alpha - pi;
            end
        end

        line = [alpha; r];
    end
end