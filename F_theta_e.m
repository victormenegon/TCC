function trapezoidal = F_theta_e(theta)

if(theta >= 0 & theta < (2*pi/3)), 
    trapezoidal = 1;
  
elseif(theta >= (2*pi/3) & theta < pi)
    trapezoidal = 1 - ((6/pi) * (theta - (2*pi/3)));
    
elseif(theta >= pi & theta < (5*pi/3))
    trapezoidal = -1;
    
elseif(theta >= (5*pi/3) & theta < (2*pi))
    trapezoidal = -1 + ((6/pi) * (theta - (5*pi/3)));
    
else 
    trapezoidal = 1;
    
end
