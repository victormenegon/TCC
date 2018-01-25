V_pwm = 10;
V_bus = 100;
gain = V_bus/V_pwm;
n = 100000;
V_ref = 50;
t = 0.00001;        %passo de calculo
V_ref = 30;
x = zeros(n,1);
y = zeros(n,1);
pwm_st = zeros(n,1);
time_lapsed = zeros(n,1);
V_barramento = zeros(n,1);

for(T = 1:n)
    if(T-1 > 0)
        
    x(T) = x(T-1)+t;
    y(T) = 50000*x(T-1);
    if(x(T) <= 0.0002)
        if(y(T) < V_ref/gain)
            pwm_st(T) = 1;
        else
            pwm_st(T) = 0;
        end
    else
        x(T) = 0;
    end
    time_lapsed(T) = T*t;
    V_barramento(T) = V_bus * pwm_st(T);
    end
end