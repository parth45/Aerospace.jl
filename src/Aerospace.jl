module Aerospace

## Structures
export
    ## Constants
    go,
    d2r,
    r2d,
    REARTH,
    WEII3,
    GO,
    AGRAV,
    GCONST,
    EARTH_MASS,
    MU,
    C20,
    FLATTENING,
    SMAJOR_AXIS,
    GW_CLONG,
    RGAS,
    KBOLTZ,
    R2D,
    D2R,
    PI,
    EPS,
    SMALL,
    ILARGE,
    LARGE,
    M2F,
    M2NMI,
    J2,
    wgs_aa,
    wgs_finv,
    wgs_ff,
    wgs_bb,
    wgs_e2,    
    
    ## Structures
    Aero6DOF,
    QuadControl,
    WGS_Pos,
    Velocity_States,
    Euler_Angles,
    MassProps,
    Angle_Rates,
    Quaternions,
    Motors,

    ## Transformations
    TR1,
    TR2,
    TR3,
    TR_BN,
    TR_NE,
    TR_EI,
    TR_WB,
    TR_VN,

    # Atmosphere
    Atmosphere,

    ## WGS-84
    ECEF2LLH,
    LLH2ECEF,
    WGS84_GRAVITY,
    
    ## Quaternion
    QuatInit,
    QuatInit!,
    QuatDerivative,
    QuatToDCM


type QuadControl
    motor1_cmd::Float64
    motor2_cmd::Float64
    motor3_cmd::Float64
    motor4_cmd::Float64
end

type Motors
    wmotor1::Float64
    wmotor2::Float64
    wmotor3::Float64
    wmotor4::Float64
end

type WGS_Pos
    time::Float64
    latitude::Float64
    longitude::Float64
    altitude::Float64
    rm_ecef::Array{Float64,1}
    rm_eci::Array{Float64,1}
    Tr_ei::Array{Float64,2}
    Tr_ne::Array{Float64,2}
end

type Velocity_States
    time_sim::Float64
    vmag::Float64
    gamma::Float64
    azimuth::Float64
    vm_ned::Array{Float64,1}
    vm_ecef::Array{Float64,1}
    vm_eci::Array{Float64,1}
end

type Quaternions
    q0::Float64
    q1::Float64
    q2::Float64
    q3::Float64
end

type Euler_Angles
    roll::Float64
    pitch::Float64
    heading::Float64
    Tr_bn::Array{Float64,2}
end    

type Angle_Rates
    wb_bi_body::Array{Float64,1}
    wb_bn_body::Array{Float64,1}
    wb_ei_eci::Array{Float64,1}
    wb_ei_ecef::Array{Float64,1}
    wb_ne_eci::Array{Float64,1}
end

type MassProps
    mass::Float64
    II::Array{Float64,2}
    cg::Array{Float64,1}
end

type Aero6DOF
    time::Float64
    dt::Float64
    position::WGS_Pos
    velocity::Velocity_States
    euler::Euler_Angles
    angle_rates::Angle_Rates
    control_vec::QuadControl
    massprops::MassProps
    quat_bi::Quaternions
    wmotor_vec::Motors
    xo::Array{Float64,1}
end

include("Utils.jl");

end # module
