P = 4;
Ke = 1;
wm = 1600;
mec_angle = 0;
ele_angle = 0;
R = 4.65;
L = 69.7*10^-3;

n = 8000;
dt = 0;
dt_1 = 0;
t = 0.001; %passo de calculo

dIa = zeros(n,1);
dIb = zeros(n,1);
dIc = zeros(n,1);
Ia = zeros(n,1);
Ib = zeros(n,1);
Ic = zeros(n,1);
Ea = zeros(n,1);
Va = zeros(n,1);
Eb = zeros(n,1);
Vb = zeros(n,1);
Ec = zeros(n,1);
Vc = zeros(n,1);
time_lapsed = zeros(n,1);


for T = 1:n,
    
    dt = dt+t;
    mec_angle = (dt*2*pi)/(60/wm);    
    ele_angle = (mec_angle*P)/2;
    ele_b = ele_angle - (2*pi/3);
    ele_c = ele_angle + (2*pi/3);
    time_lapsed(T) = dt*T;
    angulo(T) = ele_angle(T);
    
    if(ele_angle >= 0 & ele_angle < (2*pi/3))
            Ea(T) = (Ke/2)*wm*1;
            Va(T) = 4*Ea(T);
    elseif(ele_angle >= (2*pi/3) & ele_angle < pi)
            Ea(T) = (Ke/2)*wm*(1-((6/pi)*(ele_angle-(2*pi/3))));
            Va(T) = 4*Ea(T);
    elseif(ele_angle >= pi & ele_angle < (5*pi/3))
            Ea(T) = (Ke/2)*wm*(-1);
            Va(T) = 4*Ea(T);
    elseif(ele_angle >= (5*pi/3) & ele_angle < 2*pi)
            Ea(T) = (Ke/2)*wm*(-1+((6/pi)*(ele_angle-(5*pi/3))));
            Va(T) = 4*Ea(T);
    else
        %%dt = 0;
        Ea(T) = (Ke/2)*wm*1;
        Va(T) = 4*Ea(T);
    end
    
    
    for g = 1:n,
        
        if(g > 2094)
            Eb(g-2094) = Ea(g);
        else
            Eb(g+5906) = Ea(g);
        end
    end
    
     for g = 1:n,
        
        if(g > 5236)
            Ec(g-5236) = Ea(g);
        else
            Ec(g+2764) = Ea(g);
        end
    end
    
    for shift_vector_many_times = 1:7,
        for k = 1:1024;
            Eb(k+(1024*shift_vector_many_times)) = Eb(k);
            Ec(k+(1024*shift_vector_many_times)) = Ec(k);
        end
    end
    
    
    
 
    dIa(T) = dt*(Va(T) - Ea(T) - R*Ia(T))/L;
    %dIb(T) = dt*(Vb(T) - Eb(T) - R*Ib(T))/L;
    %dIc(T) = dt*(Vc(T) - Ec(T) - R*Ic(T))/L;
    
    if(T > 1),
        Ia(T) = Ia(T-1) + dIa(T);
      %  Ib(T) = Ib(T-1) + dIb(T);
      %  Ic(T) = Ic(T-1) + dIc(T);
    else 
        Ia(1) = dIa(1);
        %Ib(1) = dIb(1);
        %Ic(1) = dIc(1);
    end
   
    
end

%  plot(time_lapsed,Ea,'color', 'g');
%  hold;
%  plot(time_lapsed,Eb,'color', 'b');
