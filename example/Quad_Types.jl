
module Quad_Types

using Aerospace

export
    MotorCmd,
    Motors,
    SoftwareTelem,
    SoftwareMsg,
    Quadrotor,
    QuadForces

type MotorCmd
    motor1_cmd::Float64
    motor2_cmd::Float64
    motor3_cmd::Float64
    motor4_cmd::Float64
end

type Motors
    motor1::Actuator
    motor2::Actuator
    motor3::Actuator
    motor4::Actuator
end

type SoftwareTelem
    time::Float64
    navstates::NavStates
    axcmd::Float64
    aycmd::Float64
    azcmd::Float64
    pcmd::Float64
    qcmd::Float64
    rcmd::Float64
end

type SoftwareMsg
    time::Float64
    dt::Float64
    eta::Array{Float64,1}
    dz::Float64
    count::UInt;
    mode::UInt
    softwareTM::SoftwareTelem
    motorcmd::MotorCmd
end

type Quadrotor
    time::Float64
    dt::Float64
    airframe::Aero6DOF
    motors::Motors
    motorCmd::MotorCmd
    software::SoftwareMsg
end

type QuadForces
    f1body::Array{Float64,1}
    f2body::Array{Float64,1}
    f3body::Array{Float64,1}
    f4body::Array{Float64,1}
    m1body::Array{Float64,1}
    m2body::Array{Float64,1}
    m3body::Array{Float64,1}
    m4body::Array{Float64,1}
end



end ## End of Module
