############################################################################
#
#  Interpolation.jl
#
#  Purpose:    This file contains functions to perform linear interpolation
#              of multi-dimensional tables
#
#  Author:     Edward Daughtery
#
#  Date:       05 August 2016
#
#
############################################################################

function interp1( x, X, Y )
    ## Initialize Indicies
    ii = 1;
    VAL = 0.0;
    x1 = 0.0;
    x2 = 0.0;
    
    n1 = length(X);
    ifound = 0;
    xfac = 0;

    ## Search for X
    while (ifound == 0) 
        tmp1 = x - X[ii];
        tmp2 = x - X[ii + 1];
        tmp3 = tmp1 * tmp2;
        
        
        if (tmp3 <= 0) 
            x1 = X[ii];
            x2 = X[ii + 1];
            ifound = 1;
        else 
            ii = ii + 1;
        end
    end

    f1 = Y[ii];
    f2 = Y[ii + 1];
    
    VAL = (f1 * (x2 - x) + f2 * (x - x1)) / (x2 - x1);
    return VAL;
end

############################################################################

function interp2(x, y, X, Y, Z)

    n1 = length(X);
    n2 = length(Y);
    ifound = 1;
    jfound = 1;
    xfac = 0;
    yfac = 0;

    # Search for X
    for i = 2:n1
        if ( (x >= X[i-1]) && ( x <= X[i] ) )
            ifound = i-1;
            xfac = (x - X[i-1])/( X[i] -  X[i-1] );
            break
        end

        if (i == n1) && (ifound == 0)
            xfac = 1.0;
            ifound = i-1;
        end
        
    end

    # Search for Y
    for j = 2:n2
        if ( (y >= Y[j-1]) && ( y <= Y[j] ) )
            jfound = j-1;
            yfac = (y - Y[j-1])/( Y[j] -  Y[j-1] );
            break;
        end

        if (j > n2) && (jfound == 0)
            yfac = 1.0;
            jfound = j-1;            
        end

    end

    ZZ = (Z[ifound+1,:] - Z[ifound,:] )*xfac + Z[ifound,:];
    zout = (ZZ[jfound+1] - ZZ[jfound] )*yfac + ZZ[jfound];
    
    return zout
end

############################################################################

function interp3( x, y, z, X, Y, Z, VV )

    ##Initialize Indicies
    ii = 1;
    jj = 1;
    kk = 1;
    VAL = 0.0;
    x1 = 0.0;
    x2 = 0.0;
    y1 = 0.0;
    y2 = 0.0;
    z1 = 0.0;
    z2 = 0.0;

    ##Search for X
    ifound = 0;
    while (ifound == 0) 
        tmp1 = x - X[ii];
        tmp2 = x - X[ii + 1];
        tmp3 = tmp1 * tmp2;

        
        if (tmp3 <= 0) 
            x1 = X[ii];
            x2 = X[ii + 1];
            ifound = 1;
            
        else
            ii = ii + 1;
        end
    end

    ##Search for Y
    jfound = 0;
    while (jfound == 0) 
        tmp1 = y - Y[jj];
        tmp2 = y - Y[jj + 1];
        tmp3 = tmp1 * tmp2;


        if (tmp3 <= 0) 
            y1 = Y[jj];
            y2 = Y[jj + 1];
            ifound = 1;
        else 
            jj = jj + 1;
        end
    end

    ##Search for Z
    kfound = 0;
    while (ifound == 0)
        tmp1 = z - Z[kk];
        tmp2 = z - Z[kk + 1];
        tmp3 = tmp1 * tmp2;


        if (tmp3 <= 0) 
            z1 = Z[kk];
            z2 = Z[kk + 1];
            ifound = 1;
        else 
            kk = kk + 1;
        end
    end

    f111 = VV[ii][jj][kk];
    f211 = VV[ii + 1][jj][kk];
    f121 = VV[ii][jj + 1][kk];
    f112 = VV[ii][jj][kk + 1];
    f221 = VV[ii + 1][jj + 1][kk];
    f212 = VV[ii + 1][jj][kk + 1];
    f122 = VV[ii][jj + 1][kk + 1];
    f222 = VV[ii + 1][jj + 1][kk + 1];

    VAL = (-f112 * y * z * x2 - f112 * x * z * y2 - f112 * x * y * z1 + f112 * z * x2 * y2 + f112 * y * x2 * z1 + f112 * x * y2 * z1 - f221 * y * x1 * z2 - f221 * x * y1 * z2 + f221 * x1 * y1 * z2 + f112 * x * y * z - f221 * z * x1 * y1 - f112 * x2 * y2 * z1 + f121 * x * y * z - f121 * y * z * x2 - f121 * x * z * y1 - f121 * x * y * z2 + f121 * z * x2 * y1 + f121 * y * x2 * z2 + f121 * x * y1 * z2 - f121 * x2 * y1 * z2 + f211 * x * y * z - f211 * y * z * x1 - f212 * z * x1 * y2 - f212 * y * x1 * z1 + f212 * x * z * y2 + f212 * x * y * z1 - f111 * x * y * z + f111 * y * z * x2 + f111 * x * z * y2 + f111 * x * y * z2 - f111 * z * x2 * y2 - f111 * y * x2 * z2 - f111 * x * y2 * z2 + f111 * x2 * y2 * z2 + f211 * x * y2 * z2 - f211 * x1 * y2 * z2 - f211 * x * z * y2 - f211 * x * y * z2 + f211 * z * x1 * y2 + f211 * y * x1 * z2 + f212 * y * z * x1 + f221 * x * y * z2 + f221 * y * z * x1 + f221 * x * z * y1 - f221 * x * y * z - f212 * x * y2 * z1 + f212 * x1 * y2 * z1 - f122 * x * y1 * z1 + f122 * x2 * y1 * z1 - f212 * x * y * z - f122 * y * x2 * z1 + f122 * y * z * x2 + f122 * x * z * y1 + f122 * x * y * z1 - f122 * z * x2 * y1 - f222 * y * z * x1 - f222 * x * z * y1 - f222 * x * y * z1 + f222 * x * y * z - f122 * x * y * z - f222 * x1 * y1 * z1 + f222 * x * y1 * z1 + f222 * z * x1 * y1 + f222 * y * x1 * z1) / (-z1 + z2) / (-y2 + y1) / (-x2 + x1);

    return VAL;
end

############################################################################


