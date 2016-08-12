using Aerospace
using Base.Test

# write your own tests here
pitchi = 45.0*D2R;
yawi = 90.0*D2R;
rolli = 10.0*D2R;
Tr_bni = TR_BN(rolli, pitchi, yawi);

# Testing of the Algorithms
quat = QuatInit(rolli, pitchi, yawi);
Tr_bn = QuatToDCM(quat);
euler = ComputeEuler( Tr_bn );

# write your own tests here
@test (euler[1] -  rolli) < 1.0e-10;
@test (euler[2] -  pitchi) < 1.0e-10;
@test (euler[3] -  yawi) < 1.0e-10;
@test norm(Tr_bni - Tr_bn) < 1.0e-10;
