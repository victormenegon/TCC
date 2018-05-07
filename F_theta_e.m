function trapezoidal = F_theta_e(theta)

if(theta >= 0 && theta < (pi/18)) 
   trapezoidal = 0;
elseif(theta >= pi/18 && theta < pi/4)
    trapezoidal = theta * (36/(7*pi)) - 2/7;
elseif(theta >= pi/4 && theta < 3*pi/4)
    trapezoidal = 1;
elseif(theta >= 3*pi/4 && theta < 17*pi/18)
    trapezoidal = -theta*(36/(7*pi)) + 34/7;
elseif(theta >= 17*pi/18 && theta < 19*pi/18)
    trapezoidal = 0;
elseif(theta >= 19*pi/18 && theta < 5*pi/4)
    trapezoidal = -theta*(36/(pi*7)) + (38/7);
elseif(theta >=5*pi/4 && theta < 7*pi/4)
    trapezoidal = -1;
elseif(theta >= 7*pi/4 && theta < 35*pi/18)
    trapezoidal = theta*(36/(7*pi)) - 10;
elseif(theta >= 35*pi/18 && theta < 2*pi)
    trapezoidal = 0;
else
    trapezoidal = 0;
end
