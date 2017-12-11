function angle_normalized = normalize_angle(angle)

c_360 = 2*pi;

 if(angle >= c_360),
         angle_normalized = angle - c_360;
 elseif(angle < 0)
         angle_normalized = angle + c_360;
 else
         angle_normalized = angle;
 end