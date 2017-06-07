
module MessageTypes

using Aerospace

export
Health,
CoopFormation,
CoopProNav

type Health
    time::Float64
    icom::Bool
    igps::Bool
    num_sats::UInt8
    TAS::Float32
    IAS::Float32
    alt_hae::Float64
end

type CoopFormation
    time::Float64
    UTCtime::Float64
    rm_ecef::Array{Float64,1}
    vm_ecef::Array{Float64,1}
    rm_lead_ecef::Array{Float64,1}
    vm_lead_ecef::Array{Float64,1}
end

type CoopProNav
    time::Float64
    UTCtime::Float64
    rm_ecef::Array{Float64,1}
    vm_ecef::Array{Float64,1}
    rt_ecef::Array{Float64,1}
    vt_ecef::Array{Float64,1}
    tgo::Float64
end

end
