import numpy as np

def CrustInvKin(o, R ):
    #  Calculation of the inverse kinematics for the Crustcrawler robot
    #  Inputs :  position        o [1x3]
    #            orientation     R [3x3]
    #  Outputs : angles          q [1x4] (for the CrustCrawler robot)


    d1 = 10; #  cm (height of 2nd joint)
    a1 = 5; # (distance along "y-axis" to 2nd joint)
    a2 = 20; # (distance between 2nd and 3rd joints)
    d4 = 20; # (distance from 3rd joint to gripper center - all inclusive, ie. also 4th joint)


    #  Calculate oc
    oc = o; # - d4*R*[0;0;1] ;
    xc = oc[0], yc = oc[1], zc = oc[2]


    #  Calculate q1
    q1 = atan2(yc, xc); #  NOTE: Order of y and x.. depend on atan2 func..


    #  Calculate q2 and q3
    r2 = (xc - a1*cos(q1))^2 + (yc - a1*sin(q1))^2; # radius squared - radius can never be negative, q1 accounts for this..
    s = (zc - d1); # can be negative ! (below first joint height..)
    D = ( r2 + s^2 - a2^2 - d4^2)/(2*a2*d4);  #  Eqn. (3.44)

    q3 = atan2(-sqrt(1-D^2), D); #  Eqn. (3.46)
    q2 = atan2(s, sqrt(r2)) - atan2(d4*sin(q3), a2 + d4*cos(q3)); # Eqn. (3.47)


    #  calculate q4 - ie. rotation part
    #  r32 = R(3,2);
    #  r23 = R(2,3);
    #  c23 = cos(q2 + q3);
    #  q4 = atan2(r23/c23, r32/c23); 
    q4 = 0 # not consider rotation so far..

    q = [q1 q2 q3 q4]

    return q