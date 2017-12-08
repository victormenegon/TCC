function [trapezoidal,dt0] = F_theta_e(theta_e)

if(theta_e >= 0 & theta_e < 120), 
    trapezoidal = 1;
    dt0 = 1;
elseif(theta_e >= 120 & theta_e < 180)
    trapezoidal = (1- ((6/180) * (theta_e - 120)));
    dt0 = 1;
elseif(theta_e >= 180 & theta_e < 300)
    trapezoidal = -1;
    dt0 = 1;
elseif(theta_e >= 300 & theta_e < 360)
    trapezoidal = -1 + ((6/180) * (theta_e - 120));
    dt0 = 1;
else 
    trapezoidal = 1;
    dt0 = 0;
end
