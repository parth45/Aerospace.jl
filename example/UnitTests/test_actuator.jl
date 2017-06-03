using PyPlot
using Aerospace



# Motor Initialization
wn = 20.0;
zeta = 0.707;
rpm_pos_lim = 20.0;
rpm_neg_lim = 0.0;
rpm_rate_pos_lim = 15.0;
rpm_rate_neg_lim = -15.0;

motor1 = Initialize_Act(dt, wn, zeta,
                        rpm_pos_lim,
                        rpm_neg_lim,
                        rpm_rate_pos_lim,
                        rpm_rate_neg_lim
                        );


# Simulation Control
time = 0.0;
dt = 0.01;
tfinal = 5.0;

## Number of Sample points
nn = (tfinal - time)/dt + 1;
NN = Int64(nn);

TT = zeros(NN);
UU = zeros(NN);
X1 = zeros(NN);
X2 = zeros(NN);


for i = 1:NN
    TT[i] = time;
    X1[i] = motor1.position;
    X2[i] = motor1.rate;
    
    ucmd = 15.0;
    
    motor1 = Update_Act(motor1, ucmd);
    time = time + dt;

end

figure()
plot(TT, X1, TT, X2)
grid()
xlabel("Time (s)")
ylabel("Actuator Output")
legend(["Position", "Rate"])
