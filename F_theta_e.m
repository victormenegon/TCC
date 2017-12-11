function trapezoidal = F_theta_e(theta_e)

if(theta_e >= 0 & theta_e < 2*pi/3), 
    trapezoidal = 1;
  
elseif(theta_e >= 2*pi/3 & theta_e < pi)
    trapezoidal = 1 - ((6/pi) * (theta_e - (2*pi/3)));
    
elseif(theta_e >= pi & theta_e < 5*pi/3)
    trapezoidal = -1;
    
elseif(theta_e >= 5*pi/3 & theta_e < 2*pi)
    trapezoidal = -1 + ((6/pi) * (theta_e - (5*pi/3)));
    
else 
    trapezoidal = 1;
    
end
