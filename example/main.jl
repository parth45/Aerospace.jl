
############################################################################
#
#  main.jl
#
#  Purpose:    Main routine for Quadrotor simulation.
#
#  Author:     Edward Daughtery
#
#  Date:       05 August 2016
#
#
############################################################################
using PyPlot
using Aerospace
using Debug
include("Quad_Types.jl")
using Quad_Types
include("Software.jl")
include("Quadrotor.jl")
include("Quad_Forces.jl")

# Closing all Figures
for i = 1:100
    close()
end

# Simulation Control
time = 0.0;
dt = 0.001;
tfinal = 10.0;  

## Number of Sample points
nn = (tfinal - time)/dt + 1;
NN = UInt(nn);

TT = zeros(NN);
ALT = zeros(NN);
VMAG = zeros(NN);
GAM = zeros(NN);
AZ = zeros(NN);
ROLL = zeros(NN);
PITCH = zeros(NN);
HEADING = zeros(NN);
W1 = zeros(NN);
W2 = zeros(NN);
W3 = zeros(NN);
W4 = zeros(NN);

## Position Initialization
latitude = 0.0*d2r;
longitude = 0.0*d2r;
altitude = 0.0*d2r;
position = InitPos( time, latitude, longitude, altitude );

## Velocity Initialization
vmag = 0.0;
gamma = 0.0*d2r;
azimuth = 0.0*d2r;
velocity = InitVel( vmag, gamma, azimuth, position );

# Attitude Initialization
roll = 0.0*d2r;
pitch = 0.0*d2r;
heading = 0.0*d2r;
euler = InitEuler(roll, pitch, heading);

# Angle Rate Intialization
wb_bn_body = [0.0;0.0;0.0];
angle_rates = InitAngleRates(wb_bn_body, position, euler);

# Quadrotor Initialization
quad1 = QuadInit( time, dt, position, velocity, euler, angle_rates );

for i = 1:NN
    TT[i] = time;
    ROLL[i] = quad1.airframe.euler.roll*r2d;
    PITCH[i] = quad1.airframe.euler.pitch*r2d;
    HEADING[i] = quad1.airframe.euler.heading*r2d;
    ALT[i] = quad1.airframe.position.altitude;
    W1[i] = quad1.motors.motor1.position;
    W2[i] = quad1.motors.motor2.position;
    W3[i] = quad1.motors.motor3.position;
    W4[i] = quad1.motors.motor4.position;

    quad1 = QuadUpdate(quad1);
    time = time + dt;
end

figure()
plot(TT, ALT)
grid
xlabel("Time (s)")
ylabel("Altitude (m)")

figure()
plot(TT, ROLL)
grid
xlabel("Time (s)")
ylabel("Roll (deg)")

figure()
plot(TT, PITCH)
grid
xlabel("Time (s)")
ylabel("Pitch (deg)")

figure()
plot(TT, HEADING)
grid
xlabel("Time (s)")
ylabel("Heading (deg)")

figure()
plot(TT, W1, TT, W2, TT, W3, TT, W4)
grid
xlabel("Time (s)")
ylabel("Motor Speed (rad/s)")
legend(["Motor 1", "Motor 2", "Motor 3", "Motor 4"]);

