function [f,g] = Prob2Function(x)
    
    f = x(1)^4 - 10*x(1)^2 + x(2)^4 - 10*x(2)^2;

    % specify gradient to improve performance
    if nargout > 1
        g = [4*x(1)^3 - 20*x(1); 4*x(2)^3 - 20*x(2)];
    end
end