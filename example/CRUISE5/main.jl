
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
include("Missile_Types.jl")
include("MessageTypes.jl")
using Missile_Types
using MessageTypes
include("Software.jl")
include("CruiseMissile5.jl")

# Closing all Figures
CloseAll();

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
altitude = 4572.0;
position = InitPos( time, latitude, longitude, altitude );

## Velocity Initialization
vmag = 240.0;
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
missile1 = MissileInit( time, dt, position, velocity, euler, angle_rates );

for i = 1:NN
    # Plotting Parameters
    TT[i] = time;
    ROLL[i] = missile1.airframe.euler.roll*r2d;
    PITCH[i] = missile1.airframe.euler.pitch*r2d;
    HEADING[i] = missile1.airframe.euler.heading*r2d;
    ALT[i] = missile1.airframe.position.altitude;
    VMAG[i] = missile1.airframe.velocity.vmag;

    # Update
    missile1 = MissileUpdate(missile1);
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
