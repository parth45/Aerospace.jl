
module Missile_Types

using Aerospace

export
    Controls,
    SoftwareMsg,
    Aerodynamics,
    Missile

type Controls
    throttle::Float64
    alpha::Float64
    alphadot::Float64
    phi::Float64
    phidot::Float64
end

type SoftwareMsg
    time::Float64
    dt::Float64
    SWmode::UInt
    controls::Controls
    navstates::NavStates
    wypt_num::UInt
    axcmd::Float64
    aycmd::Float64
    azcmd::Float64
    eta_dive_cmd::Float64
    eta_turn_cmd::Float64
    tgo::Float64
    miss::Float64
    missvec_ned::Array{Float64,1}
    vel_err_int::Float64;
end

type Missile
    time::Float64
    dt::Float64
    airframe::Aero5DOF
    control::Controls
    software::SoftwareMsg
end

type Aerodynamics
    sref::Float64
    cl::Float64
    cd::Float64
    faero_body::Array{Float64,1}
end



end ## End of Module
