%E_int and E_old must be declared in your sim script as global variables
function force = Discrete_PID(kp,ki,kd,E,Tc)
    global E_int;
    global E_old;
    f_p = kp * E;

    f_d = ((E-E_old)/Tc)*kd;
    E_old = E;

    int = E*Tc; 
    E_int= E_int+int;   %numerical integration for the error
    f_i = E_int*ki;


    f_tot = f_p+f_d+f_i;
    force = f_tot;    
end