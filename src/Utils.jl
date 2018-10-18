
## Constants
go = 9.80665;
d2r = pi/180;
r2d = 180/pi;


REARTH = 6370987.308;         # Mean earth radius - m
WEII3 = 7.292115e-5;          # Angular rotation of earth - rad/s
GO = 9.80665;                 # Gravity Constant
AGRAV = 9.80675445;           # Standard value of gravity acceleration - m/s^2
GCONST = 6.673e-11;                # Universal gravitational constant - Nm^2/kg^2
EARTH_MASS = 5.973332e24;     # Mass of the earth - kg
MU = 3.9860044e14;            # Gravitational parameter=G*EARTH_MASS - m^3/s^2
C20 = -4.8416685e-4;          # Second degree zonal gravitational coefficient - ND
FLATTENING = 3.33528106e-3;   # Flattening of the Earth (WGS84) - ND
SMAJOR_AXIS = 6378137.;       # Semi-major axis of Earth's ellipsoid (WGS84) - m
GW_CLONG = 0.0;               # Greenwich celestial longitude at start of simulation - rad
RGAS = 287.053;               # Ideal gas constant - J/(K*kg)=N*m/(K*kg)
KBOLTZ = 1.38e-23;            # Boltzmann's constant - Ws/K
R2D = 57.29577951309314;      # Radians to Degrees
D2R = 0.01745329251994;       # Degrees to Radiancs


#numerical constants
PI = 3.141592653589793;       # Circumference of unit diameter circle
EPS = 1.e-10;                 # Machine precision error (type double)
SMALL = 1e-7;                 # Small real number
ILARGE = 9999;                # Large integer number
LARGE = 1e10;                 # Large real number (type double)


#conversion factors
M2F = 3.280834;               # Conversion factor m->ft
M2NMI = 5.399568e-4;          # Conversion factor  m->nm


# WGS84 Constants
J2 = -sqrt(5)*C20;
aa = 6378137.0;
finv = 298.257223563;
ff = 1.0/finv;
bb = aa*( 1.0 - ff );
e2 = 1.0 - ( 1.0 - ff )^2;

wgs_aa = aa;
wgs_finv = finv;
wgs_ff = ff;
wgs_bb = bb;
wgs_e2 = e2;


###########################################################################
## Transformation about Axis 1
function TR1(x)
  Tr1 = [
         1.0    0.0      0.0
         0.0    cos(x) sin(x)
         0.0   -sin(x) cos(x)
  ];

  return Tr1;
end

###########################################################################
## Transformation about Axis 2
function TR2(x)
  Tr2 = [
         cos(x)    0.0      -sin(x)
         0.0       1.0       0.0
         sin(x)    0.0       cos(x)
  ];

  return Tr2;
end

###########################################################################
## Transformation about Axis 3
function TR3(x)
  Tr3 = [
         cos(x)    sin(x)     0.0
         -sin(x)   cos(x)     0.0
         0.0       0.0        1.0
  ];

  return Tr3;
end

###########################################################################
## Transformation from NED to Body
function TR_BN(roll, pitch, heading)

  TRBN = TR1(roll)*TR2(pitch)*TR3(heading);
  return TRBN;

end

###########################################################################
## Transformation from ECEF to NED
function TR_NE(latitude, longitude)
  clat = cos(latitude);
  slat = sin(latitude);
  clon = cos(longitude);
  slon = sin(longitude);

  TRNE = [
          -slat*clon   -slat*slon   clat
          -slon         clon        0.0
          -clat*clon   -clat*slon   -slat
  ];
  return TRNE;
end

###########################################################################
## Transformation from ECI to ECEF
function TR_EI(time)
  TREI = TR3(WEII3*time);
  return TREI;
end

###########################################################################
## Transformation from Body to Wind Skid To Turn
function TR_WB(alpha, beta)
  TRWB = TR3(beta)*TR2(-alpha);
  return TRWB;
end

###########################################################################
## Transformation from Body to Wind Skid To Turn
function TR_VN(gam, azi)
    TRVN = zeros(3,3);
    TRVN[1,1] = cos(gam)*cos(azi);
    TRVN[1,2] = cos(gam)*sin(azi);
    TRVN[1,3] = -sin(gam);
    TRVN[2,1] = -sin(azi);
    TRVN[2,2] = cos(azi);
    TRVN[2,3] = 0.0;
    TRVN[3,1] = sin(gam)*cos(azi);
    TRVN[3,2] = sin(gam)*sin(azi);
    TRVN[3,3] = cos(gam);

  return TRVN;
end

###########################################################################
## Transformation from Body to Wind Skid To Turn
function TR_BV( alfa,  phii)
  ca = cos(alfa);
  sa = sin(alfa);
  cp = cos(phii);
  sp = sin(phii);

  TBV = zeros(3,3);
  TBV[1,1] = ca;
  TBV[1,2] = sa*sp;
  TBV[1,3] = -sa*cp;
  TBV[2,1] = 0.0;
  TBV[2,2] = cp;
  TBV[2,3] = sp;
  TBV[3,1] = sa;
  TBV[3,2] = -ca*sp;
  TBV[3,3] = cp*ca;

  return TBV;
end
###########################################################################
# Atmosphere Function from Zipfel's book
function Atmosphere(alt, vmag)

  # Universal Air Constant
  Rair = 287.058;

  # Atmosphere Parameters
  if( alt < 11000.0 )
    Temp = 288.15 - 0.0065*alt;
    Press = 101325.0*(Temp/288.15)^5.2559;
  else
    Temp = 216.0;
    Press = 22630.0*exp(-0.00015769*(alt - 11000.0));
  end
  rho = Press/(Rair*Temp);
  aa = sqrt(1.4*Rair*Temp);

  # Mach and Dynamic Pressure
  Mach = vmag/aa;
  qbar = 0.5*rho*vmag*vmag;

  return [Press, Temp, rho, aa, Mach, qbar];

end


###########################################################################
function ECEF2LLH(rm_ecef)

  X = rm_ecef[1];
  Y = rm_ecef[2];
  Z = rm_ecef[3];

  # Longitude
  lon = atan(Y,X);

  # Eccentricity
  ee = e2*(aa/bb)^2;

  p = sqrt( X.*X + Y.*Y);
  r = sqrt( p.*p + Z.*Z);
  u = atan( bb.*Z.*( 1 + ee.*bb./r)./(aa.*p));

  # Latitude
  lat = atan( (Z + ee.*bb.*sin(u).^3)./( p - e2.*aa.*cos(u).^3) );

  v = aa./sqrt( 1.0 - e2.*sin(lat).^2 );

  # Altitude
  alt = p.*cos(lat) + Z.*sin(lat) - aa*aa./v;

  return [lat lon alt];
end

###########################################################################
function LLH2ECEF(lat, lon, alt)

  vv = aa/sqrt( 1.0 - e2*sin(lat)*sin(lat) );
  xx = ( vv + alt )*cos(lat)*cos(lon);
  yy = ( vv + alt )*cos(lat)*sin(lon);
  zz = ( vv*( 1.0 - e2 ) + alt)*sin(lat);

  rm_ecef = [xx;yy;zz];
  return rm_ecef;
end

###########################################################################
function WGS84_GRAVITY(rm_ecef)
  rmag = norm(rm_ecef);
  px = rm_ecef[1];
  py = rm_ecef[2];
  pz = rm_ecef[3];
  tmp1 = pz/rmag;

  vec = [
          ( 1.0 + 1.5*J2*(aa/rmag)^2*(1 - 5*tmp1^2) )*px/rmag
          ( 1.0 + 1.5*J2*(aa/rmag)^2*(1 - 5*tmp1^2) )*py/rmag
          ( 1.0 + 1.5*J2*(aa/rmag)^2*(3 - 5*tmp1^2) )*pz/rmag
  ];

  # Raw Gravity Vector
  GRAV_ECEF = -MU/( rmag^2 )*vec

  # Earth Rotation Vector
  wei_ecef = [0.0;0.0;WEII3];

  # Effective Gravity Vector
  grav_ecef = GRAV_ECEF - cross( wei_ecef , cross( wei_ecef, rm_ecef ) );

  return grav_ecef;
end

###########################################################################
function QuatInit(roll, pitch, heading)
  q0 = cos(heading/2)*cos(pitch/2)*cos(roll/2) + sin(heading/2)*sin(pitch/2)*sin(roll/2);
  q1 = cos(heading/2)*cos(pitch/2)*sin(roll/2) - sin(heading/2)*sin(pitch/2)*cos(roll/2);
  q2 = cos(heading/2)*sin(pitch/2)*cos(roll/2) + sin(heading/2)*cos(pitch/2)*sin(roll/2);
  q3 = sin(heading/2)*cos(pitch/2)*cos(roll/2) - cos(heading/2)*sin(pitch/2)*sin(roll/2);

  quat = [q0;q1;q2;q3];

  # Normalization
  quat = quat/norm(quat);

  return quat;
end
###########################################################################
function QuatInit( Tr_bi::Array{Float64,2} )
  t11 = Tr_bi[1,1];
  t12 = Tr_bi[1,2];
  t13 = Tr_bi[1,3];
  t23 = Tr_bi[2,3];
  t33 = Tr_bi[3,3];

  pitch = asin(-t13);
  heading = atan(t12,t11);
  roll = atan(t23,t33);

  q0 = cos(heading/2)*cos(pitch/2)*cos(roll/2) + sin(heading/2)*sin(pitch/2)*sin(roll/2);
  q1 = cos(heading/2)*cos(pitch/2)*sin(roll/2) - sin(heading/2)*sin(pitch/2)*cos(roll/2);
  q2 = cos(heading/2)*sin(pitch/2)*cos(roll/2) + sin(heading/2)*cos(pitch/2)*sin(roll/2);
  q3 = sin(heading/2)*cos(pitch/2)*cos(roll/2) - cos(heading/2)*sin(pitch/2)*sin(roll/2);

  quat = [q0;q1;q2;q3];

  # Normalization
  quat = quat/norm(quat);

  return quat;
end


###########################################################################

function QuatDerivative(quat::Array{Float64,1}, wb_body::Array{Float64,1} )
  pp = wb_body[1];
  qq = wb_body[2];
  rr = wb_body[3];

  lam = 1.0 - norm(quat)^2;

  BB = [
        0   -pp   -qq    -rr
        pp   0     rr    -qq
        qq  -rr    0      pp
        rr   qq   -pp     0
  ];

  kgain = 100.0;
  quatdot = 0.5*BB*quat + kgain*lam*quat;

  return quatdot;
end

###########################################################################

function QuatToDCM(quat)

  q0 = quat[1];
  q1 = quat[2];
  q2 = quat[3];
  q3 = quat[4];

  Tr_bi = zeros(3,3);
  Tr_bi[1,1] = q0^2 + q1^2 - q2^2 - q3^2;
  Tr_bi[2,2] = q0^2 - q1^2 + q2^2 - q3^2;
  Tr_bi[3,3] = q0^2 - q1^2 - q2^2 + q3^2;

  Tr_bi[2,1] = 2.0*(q1*q2 - q0*q3);
  Tr_bi[1,2] = 2.0*(q1*q2 + q0*q3);

  Tr_bi[3,1] = 2.0*(q1*q3 + q0*q2);
  Tr_bi[1,3] = 2.0*(q1*q3 - q0*q2);

  Tr_bi[3,2] = 2.0*(q2*q3 - q0*q1);
  Tr_bi[2,3] = 2.0*(q2*q3 + q0*q1);

  return Tr_bi';
end

###########################################################################
function ComputeEuler( Tr_bi::Array{Float64,2} )

  t11 = Tr_bi[1,1];
  t12 = Tr_bi[1,2];
  t13 = Tr_bi[1,3];
  t23 = Tr_bi[2,3];
  t33 = Tr_bi[3,3];

  pitch = asin(-t13);
  heading = atan(t12,t11);
  roll = atan(t23,t33);

  euler = [roll; pitch; heading];

  return euler;
end

###########################################################################
function Pause()
  print("Program Paused:  Press any key to Continue, or press q to quit: ");
  xx = readline();
  if xx == "q\r\n"
    error("Terminated Program");
  end
  return
end

###########################################################################
function CloseAll()

  for i = 1:500
    close();
  end

  return
end

###########################################################################
function Bound(xx::Float64, xmin::Float64, xmax::Float64)
  yy = xx;

  if (xx > xmax)
    yy = xmax;
  elseif(xx < xmin)
    yy = xmin;
  else
    yy = xx;
  end

  return yy;
end
