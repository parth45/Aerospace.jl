
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
include("Quad_Types.jl")
using Quad_Types
include("Software.jl")
include("Quadrotor.jl")
include("Quad_Forces.jl")

# Simulation Control
time = 0.0;
dt = 0.01;
tfinal = 100.0;  

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
    ROLL[i] = quad1.airframe.euler.roll;
    PITCH[i] = quad1.airframe.euler.pitch;
    HEADING[i] = quad1.airframe.euler.heading;
    ALT[i] = quad1.airframe.position.altitude;

    quad1 = QuadUpdate(quad1);
    time = time + dt;
end

