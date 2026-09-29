This appendix contains the remainder of the data for the F-16 aircraft model given
in Chapter 3. The usage of the lookup tables will be made evident by referring to the
aircraft model. These data, Appendix B, and the other programs used in this book can
be obtained on a floppy disc, at a nominal cost, from Dr. B. L. Stevens, 1051 Park
Manor Terr., Marietta, GA 30064.
A.1 Mass Properties
Weight (lbs) ∶
 W = 20, 500
Moment of Intertia (slug-ft2 ) ∶
 Jxx = 9, 456
Jyy = 55, 814
Jzz = 63, 100
Jxz =
 982
A.2 Wing Dimensions
Span = 30 ft
Area = 300 ft2
m.a.c = 11.32 ft
A.3 Reference CG Location
Xcg = 0.35c

A.4 Control Surface Actuator Models
deflection limit
 rate limit
Elevator
 ±25.0∘ ,
 60 ∘ ∕s,
Ailerons
 ±21.5∘ ,
 80 ∘ ∕s,
Rudder
 ±30.0∘ ,
 120∘ ∕s,

time const.
0.0495 s lag
0.0495 s lag
0.0495 s lag
A.5 Engine Angular Momentum
Assumed fixed at 160 slug-ft2/s
A.6 Standard Atmosphere Model
SUBROUTINE ADC(VT,ALT,AMACH,QBAR)
DATA R0/2.377E-3/
TFAC = 1.0 - 0.703E-5 * ALT
T
 = 519.0 * TFAC
IF (ALT .GE. 35000.0) T= 390.0
RHO = R0 * (TFAC**4.14)
AMACH= VT/SQRT(1.4*1716.3*T)
QBAR = 0.5*RHO*VT*VT
C
 PS
 = 1715.0 * RHO * T
RETURN
END
! air data computer
! sea-level density
! temperature
! density
! Mach number
! dynamic pressure
! static pressure
A.7 Engine Model
The F-16 engine power response is modeled by a first-order lag (in function PDOT,
given below). The rest of the model consists of the throttle gearing (in TGEAR) and
the lookup tables for thrust as a function of operating power level, altitude, and Mach
(in THRUST). In the thrust lookup tables the rows correspond to a Mach number vari-
ation from 0 to 1.0 in increments of 0.2, and the columns correspond to altitudes from
0 to 50,000 ft in increments of 10,000 ft. There is a table for each of the power levels
“idle,” “military,” and “maximum.” The accompanying linear interpolation algorithm
can extrapolate beyond the boundaries of a table, but the results may not be realistic.
FUNCTION TGEAR(THTL) ! Power command v. thtl. relationship
IF(THTL.LE.0.77) THEN
TGEAR = 64.94*THTL
ELSE
TGEAR = 217.38*THTL-117.38
END IF
RETURN
END
FUNCTION PDOT(P3,P1) ! PDOT= rate of change of power
IF (P1.GE.50.0) THEN ! P3= actual power, P1= power command
IF (P3.GE.50.0) THEN
T=5.0
P2=P1
ELSE
P2=60.0
T=RTAU(P2-P3)
END IF
ELSE
IF (P3.GE.50.0) THEN
T=5.0
P2=40.0
ELSE
P2=P1
T=RTAU(P2-P3)
END IF
END IF
PDOT=T*(P2-P3)
RETURN
END
FUNCTION RTAU(DP)
 ! used by function PDOT
IF (DP.LE.25.0) THEN
RTAU=1.0
 ! reciprocal time constant
ELSE IF (DP.GE.50.0)THEN
RTAU=0.1
ELSE
RTAU=1.9-.036*DP
END IF
RETURN
END
FUNCTION THRUST(POW,ALT,RMACH)
 ! Engine thrust model
REAL A(0:5,0:5), B(0:5,0:5), C(0:5,0:5)
DATA A/
+ 1060.0, 670.0, 880.0, 1140.0, 1500.0, 1860.0,
+ 635.0, 425.0, 690.0, 1010.0, 1330.0, 1700.0,
+ 60.0, 25.0, 345.0, 755.0, 1130.0, 1525.0,
+ -1020.0, -710.0, -300.0, 350.0, 910.0, 1360.0,
+ -2700.0, -1900.0, -1300.0, -247.0,
 600.0, 1100.0,
+ -3600.0, -1400.0, -595.0, -342.0,
 -200.0,
 700.0/
C
 mil data now
DATA B/
+ 12680.0, 9150.0, 6200.0, 3950.0, 2450.0, 1400.0, + 12680.0,
9150.0, 6313.0, 4040.0, 2470.0, 1400.0, + 12610.0, 9312.0, 6610.0,
4290.0, 2600.0, 1560.0, + 12640.0, 9839.0, 7090.0, 4660.0, 2840.0,
1660.0, + 12390.0, 10176.0, 7750.0, 5320.0, 3250.0, 1930.0,
+ 11680.0, 9848.0, 8050.0, 6100.0, 3800.0, 2310.0/
C
 max data now
DATA C/
+ 20000.0, 15000.0, 10800.0, 7000.0, 4000.0, 2500.0,
+ 21420.0, 15700.0, 11225.0, 7323.0, 4435.0, 2600.0,
+ 22700.0, 16860.0, 12250.0, 8154.0, 5000.0, 2835.0,
+ 24240.0, 18910.0, 13760.0, 9285.0, 5700.0, 3215.0,
+ 26070.0, 21075.0, 15975.0, 11115.0, 6860.0, 3950.0,
+ 28886.0, 23319.0, 18300.0, 13484.0, 8642.0, 5057.0/
C
H = .0001*ALT
I = INT(H)
IF(I.GE.5)I=4
DH= H-FLOAT(I)
RM= 5.0*RMACH
M = INT(RM)
IF(M.GE.5)M=4
DM= RM-FLOAT(M)
CDH=1.0-DH
S= B(I,M) *CDH + B(I+1,M) *DH
T= B(I,M+1)*CDH + B(I+1,M+1)*DH
TMIL= S + (T-S)*DM
IF( POW .LT. 50.0 ) THEN
S= A(I,M) *CDH + A(I+1,M) *DH
T= A(I,M+1)*CDH + A(I+1,M+1)*DH
TIDL= S + (T-S)*DM
THRUST=TIDL+(TMIL-TIDL)*POW*.02
ELSE
S= C(I,M) *CDH + C(I+1,M) *DH
T= C(I,M+1)*CDH + C(I+1,M+1)*DH
TMAX= S + (T-S)*DM
THRUST=TMIL+(TMAX-TMIL)*(POW-50.0)*.02
END IF
RETURN
END

A.8 Aerodynamic Data
The aerodynamic data tables and associated interpolation algorithms, given below,
will provide values for the body-axis dimensionless aerodynamic coefficients of the
F-16 model at arbitrary values of the independent variables. The angle-of-attack range
of the tables is from −10∘ to 45∘ in 5∘ increments, and the sideslip angle range is from
−30∘ to 30∘ in either 5 ∘ or 10∘ increments. The given interpolation algorithm interpo-
lates linearly between the data points; it will extrapolate beyond the table boundaries,
but the results may be unrealistic.
SUBROUTINE DAMP(ALPHA, D) ! various damping coefficients
REAL A(-2:9,9),D(9)
DATA A/
& -.267, -.110, .308,
 1.34, 2.08, 2.91, 2.76,
&
 2.05, 1.50, 1.49,
 1.83, 1.21,
&
 .882, .852, .876,
 .958, .962, .974, .819,
&
 .483, .590, 1.21, -.493, -1.04,
& -.108, -.108, -.188,
 .110, .258, .226, .344,
&
 .362, .611, .529,
 .298, -2.27,
& -8.80, -25.8, -28.9, -31.4, -31.2, -30.7, -27.7,
& -28.2, -29.0, -29.8, -38.3, -35.3,
& -.126, -.026, .063,
 .113, .208, .230, .319,
&
 .437, .680, .100,
 .447, -.330,
& -.360, -.359, -.443, -.420, -.383, -.375, -.329,
& -.294, -.230, -.210, -.120, -.100,
& -7.21, -.540, -5.23, -5.26, -6.11, -6.64, -5.69,
& -6.00, -6.20, -6.40, -6.60, -6.00,
& -.380, -.363, -.378, -.386, -.370, -.453, -.550,
& -.582, -.595, -.637, -1.02, -.840,
&
 .061, .052, .052, -.012, -.013, -.024, .050,
&
 .150, .130, .158,
 .240, .150/
C
S= 0.2 * ALPHA
K= INT(S)
IF(K .LE. -2) K= -1
IF(K .GE. 9) K= 8
DA= S - FLOAT(K)
L = K + INT( SIGN(1.1,DA) )
DO 1, I= 1,9
1
 D(I)= A(K,I) + ABS(DA) * (A(L,I) - A(K,I))
END
C
C D1= CXq; D2= CYr; D3= CYp; D4= CZq; D5= Clr; D6= Clp
C D7= Cmq; D8= Cnr; D9= Cnp
FUNCTION CX(ALPHA,EL) ! x-axis aerodynamic force coeff.
REAL A(-2:9,-2:2)
DATA A/
& -.099, -.081, -.081, -.063, -.025, .044, .097,
& .113, .145, .167, .174, .166,
& -.048, -.038, -.040, -.021, .016, .083, .127,
& .137, .162, .177, .179, .167,
& -.022, -.020, -.021, -.004, .032, .094, .128,
& .130, .154, .161, .155, .138,
& -.040, -.038, -.039, -.025, .006, .062, .087,
& .085, .100, .110, .104, .091,
& -.083, -.073, -.076, -.072, -.046, .012, .024,
& .025, .043, .053, .047, .040/
C
S= 0.2 * ALPHA
K= INT(S)
IF(K .LE. -2) K= -1
IF(K .GE. 9) K= 8
DA= S - FLOAT(K)
L = K + INT( SIGN(1.1,DA) )
S= EL/12.0
M= INT(S)
IF(M .LE. -2) M= -1
IF(M .GE. 2) M= 1
DE= S - FLOAT(M)
N= M + INT( SIGN(1.1,DE) )
T= A(K,M)
U= A(K,N)
V= T + ABS(DA) * (A(L,M) - T)
W= U + ABS(DA) * (A(L,N) - U)
CX= V + (W-V) * ABS(DE)
RETURN
END
FUNCTION CY(BETA,AIL,RDR) ! sideforce coefficient
CY= -.02*BETA + .021*(AIL/20.0) + .086*(RDR/30.0)
END
FUNCTION CZ(ALPHA,BETA,EL) ! z-axis force coeff.
REAL A(-2:9)
DATA A/ .770, .241, -.100, -.416, -.731, -1.053,
& -1.366, -1.646, -1.917, -2.120, -2.248, -2.229/
S= 0.2 * ALPHA
K= INT(S)
IF(K .LE. -2) K= -1
IF(K .GE. 9) K= 8
DA= S - FLOAT(K)
L = K + INT( SIGN(1.1,DA) )
S= A(K) + ABS(DA) * (A(L) - A(K))
CZ= S*(1-(BETA/57.3)**2) - .19*(EL/25.0)
END
FUNCTION CM(ALPHA,EL) ! pitching moment coeff.
REAL A(-2:9,-2:2)
DATA A/
& .205, .168, .186, .196, .213, .251, .245,
& .238, .252, .231, .198, .192,
& .081, .077, .107, .110, .110, .141, .127,
& .119,
 .133, .108, .081, .093,
& -.046, -.020, -.009, -.005, -.006, .010, .006,
& -.001, .014, .000, -.013, .032,
& -.174, -.145, -.121, -.127, -.129, -.102, -.097,
& -.113, -.087, -.084, -.069, -.006,
& -.259, -.202, -.184, -.193, -.199, -.150, -.160,
& -.167, -.104, -.076, -.041, -.005/
C
 SAME INTERPOLATION AS CX ********************
C
FUNCTION CL(ALPHA,BETA) ! rolling moment coeff.
REAL A(-2:9,0:6)
DATA A/12*0,
& -.001, -.004, -.008, -.012, -.016, -.019, -.020,
& -.020, -.015, -.008, -.013, -.015,
& -.003, -.009, -.017, -.024, -.030, -.034, -.040,
& -.037, -.016, -.002, -.010, -.019,
& -.001, -.010, -.020, -.030, -.039, -.044, -.050,
& -.049, -.023, -.006, -.014, -.027,
& .000, -.010, -.022, -.034, -.047, -.046, -.059,
& -.061, -.033, -.036, -.035, -.035,
& .007, -.010, -.023, -.034, -.049, -.046, -.068,
& -.071, -.060, -.058, -.062, -.059,
& .009, -.011, -.023, -.037, -.050, -.047, -.074,
& -.079, -.091, -.076, -.077, -.076/
C
S= 0.2 * ALPHA
K= INT(S)
IF(K .LE. -2) K= -1
IF(K .GE. 9) K= 8
DA= S - FLOAT(K)
L = K + INT( SIGN(1.1,DA) )
S= .2* ABS(BETA)
M= INT(S)
IF(M .EQ. 0) M= 1
IF(M .GE. 6) M= 5
DB= S - FLOAT(M)
N= M + INT( SIGN(1.1,DB) )
T= A(K,M)
U= A(K,N)
V= T + ABS(DA) * (A(L,M) - T)
W= U + ABS(DA) * (A(L,N) - U)
DUM= V + (W-V) * ABS(DB)
CL= DUM + SIGN(1.0, BETA)
RETURN
END
FUNCTION CN(ALPHA,BETA)
 ! yawing moment coeff.
REAL A(-2:9,0:6)
DATA A/12 *0,
& .018, .019, .018, .019, .019, .018, .013,
& .007, .004, -.014, -.017, -.033,
& .038, .042, .042, .042, .043, .039, .030,
& .017, .004, -.035, -.047, -.057,
& .056, .057, .059, .058, .058, .053, .032,
& .012, .002, -.046, -.071, -.073,
& .064, .077, .076, .074, .073, .057, .029,
& .007, .012, -.034, -.065, -.041,
& .074, .086, .093, .089, .080, .062, .049,
& .022, .028, -.012, -.002, -.013,
& .079, .090, .106, .106, .096, .080, .068,
& .030, .064, .015, .011, -.001/
C NOW USE SAME INTERPOLATION AS CL ********************
C
FUNCTION DLDA(ALPHA,BETA) ! rolling mom. due to ailerons
REAL A(-2:9,-3:3)
DATA A/-.041, -.052, -.053, -.056, -.050, -.056, -.082,
& -.059, -.042, -.038, -.027, -.017,
& -.041, -.053, -.053, -.053, -.050, -.051, -.066,
& -.043, -.038, -.027, -.023, -.016,
& -.042, -.053, -.052, -.051, -.049, -.049, -.043,
& -.035, -.026, -.016, -.018, -.014,
& -.040, -.052, -.051, -.052, -.048, -.048, -.042,
& -.037, -.031, -.026, -.017, -.012,
& -.043, -.049, -.048, -.049, -.043, -.042, -.042,
& -.036, -.025, -.021, -.016, -.011,
& -.044, -.048, -.048, -.047, -.042, -.041, -.020,
& -.028, -.013, -.014, -.011, -.010,
& -.043, -.049, -.047, -.045, -.042, -.037, -.003,
& -.013, -.010, -.003, -.007, -.008/
S= 0.2 * ALPHA
K= INT(S)
IF(K .LE. -2) K= -1
IF(K .GE. 9) K= 8
DA= S - FLOAT(K)
L = K + INT( SIGN(1.1,DA) )
S= 0.1 * BETA
M= INT(S)
IF(M .eq. -3) M= -2
IF(M .GE. 3) M= 2
DB= S - FLOAT(M)
N= M + INT( SIGN(1.1,DB) )
T= A(K,M)
U= A(K,N)
V= T + ABS(DA) * (A(L,M) - T)
W= U + ABS(DA) * (A(L,N) - U)
DLDA= V + (W-V) * ABS(DB)
RETURN
END
FUNCTION DLDR(ALPHA,BETA) ! rolling moment due to rudder
REAL A(-2:9,-3:3) ! use same interpolation as DLDA
DATA A/ .005, .017, .014, .010, -.005, .009, .019,
& .005, -.000, -.005, -.011, .008,
& .007, .016, .014, .014, .013, .009, .012,
& .005, .000, .004, .009, .007,
& .013, .013, .011, .012, .011, .009, .008,
& .005, -.002, .005, .003, .005,
& .018, .015, .015, .014, .014, .014, .014,
& .015, .013, .011, .006, .001,
& .015, .014, .013, .013, .012, .011, .011,
& .010, .008, .008, .007, .003,
& .021, .011, .010, .011, .010, .009, .008,
& .010, .006, .005, .000, .001,
& .023, .010, .011, .011, .011, .010, .008,
& .010, .006, .014, .020, .000/
C
FUNCTION DNDA(ALPHA,BETA) ! yawing moment due to ailerons
REAL A(-2:9,-3:3) ! use same interpolation as DLDA *
DATA A/ .001, -.027, -.017, -.013, -.012, -.016, .001,
& .017,
 .011,
 .017,
 .008,
 .016,
& .002, -.014, -.016, -.016, -.014,
 -.019,
 -.021,
& .002,
 .012,
 .015,
 .015,
 .011,
& -.006, -.008, -.006, -.006, -.005,
 -.008, -.005,
& .007,
 .004,
 .007,
 .006,
 .006,
& -.011, -.011, -.010, -.009, -.008,
 -.006,
 .000,
& .004,
 .007,
 .010,
 .004,
 .010,
& -.015, -.015, -.014, -.012, -.011,
 -.008,
 -.002,
& .002,
 .006,
 .012,
 .011,
 .011,
& -.024, -.010, -.004, -.002, -.001,
 .003,
 .014,
& .006, -.001,
 .004,
 .004,
 .006,
& -.022,
 .002, -.003, -.005, -.003,
 -.001, -.009,
& -.009, -.001,
 .003, -.002,
 .001/
C
FUNCTION DNDR(ALPHA,BETA) ! yawing moment due to rudder
REAL A(-2:9,-3:3)
DATA A/ -.018, -.052, -.052, -.052, -.054, -.049, -.059,
& -.051, -.030, -.037, -.026, -.013,
& -.028, -.051, -.043, -.046, -.045, -.049, -.057,
& -.052, -.030, -.033, -.030, -.008,
& -.037, -.041, -.038, -.040, -.040, -.038, -.037,
& -.030, -.027, -.024, -.019, -.013,
& -.048, -.045, -.045, -.045, -.044, -.045, -.047,
& -.048, -.049, -.045, -.033, -.016,
& -.043, -.044, -.041, -.041, -.040, -.038, -.034,
& -.035, -.035, -.029, -.022, -.009,
& -.052, -.034, -.036, -.036, -.035, -.028, -.024,
& -.023, -.020, -.016, -.010, -.014,
& -.062, -.034, -.027, -.028, -.027, -.027, -.023,
& -.023, -.019, -.009, -.025, -.010/
C NOW USE SAME INTERPOLATION AS DLDA ********************



APPENDIX B
SOFTWARE
This appendix contains the Fortran code that is required to use the aircraft models
given in the text and is not otherwise readily available. For the steady-state trim algo-
rithm (Section B.1) we give the basic trimmer subroutine, part of the constraint sub-
routine, a cost function, and the Simplex minimization algorithm. The user must write
a driver program and add additional flight-path constraints, as required. In Section B.2
a subroutine for numerical linearization is given, and the user need only add a driver
program. Software for time-history simulation and control systems design is readily
available from other sources and so, in the rest of this appendix, we have given only
the Runge-Kutta algorithm that was used for most of the examples.
Appendix A, Appendix B, and the programs used in this book can be obtained
on a floppy disc, at a nominal cost, from Dr. B. L. Stevens, 1051 Park Manor Terr.,
Marietta, GA 30064.
B.1 AIRCRAFT STEADY-STATE TRIM CODE
The subroutine “TRIMMER” (below) sets up a function minimization algorithm to
determine a steady-state trim condition for either a 6-DoF or 3-DoF (longitudinal-
only) aircraft model. The subroutine arguments are the number of degrees of freedom
(NV) and the “COST” function (which must be declared “EXTERNAL” in the main
program). Labeled COMMON storage is used to pass the state and control vectors
to and from the main program and the cost function (and, in the case of the control
vector, the aircraft model also).
The main program must initialize the state vector according to the trim condition
required, and the control vector can simply be set to zero initially. It must also set the
turn rate, roll rate, or pull up rate; set flags for coordinated turns or stability-axis roll;
and pass these through a common block (“CONSTRNT”) to the constraint routine.
A Simplex routine (given below) is used for function minimization, and it returns the
coordinates of the cost function minimum in the Simplex vector S. The cost function
is then called once more to set the state and control vectors to their final values, and
these values are passed through COMMON to the main program to be placed in a
data file. Subroutine “SMPLX” can easily be replaced by “ZXMWD” from the IMSL
library, or “AMOEBA” from “Numerical Recipes” if desired. The author is indebted
to Dr. P. Vesty for this Simplex routine.
SUBROUTINE TRIMMER (NV, COST)
PARAMETER (NN=20, MM=10)
EXTERNAL COST
CHARACTER*1 ANS
DIMENSION S(6), DS(6)
COMMON/ STATE/ X(NN)
COMMON/ CONTROLS/ U(MM)
COMMON/ OUTPUT/ AN,AY,AX,QBAR,AMACH ! common to aircraft
DATA RTOD /57.29577951/
S(1)= U(1)
S(2)= U(2)
S(3)= X(2)
IF(NV .LE. 3) GO TO 10
S(4)= U(3)
S(5)= U(4)
S(6)= X(3)
10
 DS(1) = 0.2
DS(2)= 1.0
DS(3)= 0.02
IF(NV .LE. 3) GO TO 20
DS(4)= 1.0
DS(5)= 1.0
DS(6)= 0.02
20
 NC= 1000
WRITE(*,’(1X,A,$)’)’Reqd. # of trim iterations (def. = 1000) : ’
READ(*,*,ERR=20) NC
SIGMA = -1.0
CALL SMPLX(COST,NV,S,DS,SIGMA,NC,F0,FFIN)
FFIN = COST(S)
IF (NV .GT. 3) THEN
WRITE(*,’(/11X,A)’)’Throttle
 Elevator,
 Ailerons,
 Rud-
der’
WRITE(*,’(9X,4(1PE10.2,3X),/)’) U(1), U(2), U(3), U(4)
WRITE(*,99)’Angle of attack’,RTOD*X(2),’Sideslip angle’,RTOD*X(3)
WRITE(*,99) ’Pitch angle’, RTOD*X(5), ’Bank angle’, RTOD*X(4)
WRITE(*,99) ’Normal acceleration’, AN, ’Lateral acceln’, AY
WRITE(*,99) ’Dynamic pressure’, QBAR, ’Mach number’, AMACH
ELSE
WRITE(*,’(/1X,A)’)’ Throttle
 Elevator
 Alpha Pitch’
WRITE(*,’(1X,4(1PE10.2,3X))’)U(1),U(2),X(2)*RTOD,X(3)*RTOD
WRITE(*,’(/1X,A)’)’Normal acceleration Dynamic Pressure Mach ’
WRITE(*,’(5X,3(1PE10.2,7X))’) AN,QBAR,AMACH
END IF
WRITE(*,99)’Initial cost function ’,F0,’Final cost function’,FFIN
99
 FORMAT(2(1X,A22,1PE10.2))
40
 WRITE(*,’(/1X,A,$)’) ’More Iterations ? (def= Y) : ’
READ(*,’(A)’,ERR= 40) ANS
IF (ANS .EQ. ’Y’.OR. ANS .EQ. ’y’.OR. ANS .EQ. ’/’) GO TO 10
IF (ANS .EQ. ’N’.OR. ANS .EQ. ’n’) RETURN
GO TO 40
END
FUNCTION CLF16 (S)
 ! F16 cost function (see text)
PARAMETER (NN=20)
REAL S(*), XD(NN)
COMMON/STATE/X(NN) ! common to main program
COMMON/CONTROLS/THTL,EL,AIL,RDR ! to aircraft
THTL = S(1)
EL = S(2)
X(2)= S(3)
AIL = S(4)
RDR = S(5)
X(3) = S(6)
X(13)= TGEAR (THTL)
CALL CONSTR (X)
CALL
 F (TIME,X,XD)
CLF16 = XD(1)**2 + 100.0*( XD(2)**2 + XD(3)**2 )
&
 + 10.0*( XD(7)**2 + XD(8)**2 + XD(9)**2 )
RETURN
END
SUBROUTINE CONSTR (X) ! used by COST, to apply constraints
DIMENSION X(*)
LOGICAL COORD, STAB
COMMON/CNSTRNT/RADGAM,SINGAM,RR,PR,TR,PHI,CPHI,SPHI,COORD,STAB
C common to main program.
CALPH = COS(X(2))
SALPH = SIN(X(2))
CBETA = COS(X(3))
SBETA = SIN(X(3))
IF (COORD) THEN
! coordinated turn logic here
ELSE IF (TR .NE. 0.0) THEN
! skidding turn logic here
ELSE
 ! non-turning flight
X(4)= PHI
D = X(2)
IF(PHI .NE. 0.0) D = -X(2) ! inverted
IF( SINGAM .NE. 0.0 ) THEN ! climbing
SGOCB = SINGAM / CBETA
X(5)= D + ATAN( SGOCB/SQRT(1.0-SGOCB*SGOCB)) ! roc constraint
ELSE
X(5) = D
 ! level
END IF
X(7)= RR
X(8)= PR
IF (STAB) THEN
 ! stab.-axis roll
X(9)= RR*SALPH/CALPH
ELSE
X(9) = 0.0
 ! body-axis roll
END IF
END IF
RETURN
END
SUBROUTINE SMPLX(FX,N,X,DX,SD,M,Y0,YL)
C This simplex algorithm minimizes FX(X), where X is (Nx1).
C DX contains the initial perturbations in X. SD should be set
according
C to the tolerance required; when SD<0 the algorithm calls FX M
times
REAL X(*), DX(*)
DIMENSION XX(32), XC(32), Y(33), V(32,32)
C
NV=N+1
DO 2 I=1,N
DO 1 J=1,NV
1
 V(I,J)=X(I)
2
 V(I,I+1)=X(I)+DX(I)
Y0=FX(X)
Y(1)=Y0
DO 3 J=2,NV
3
 Y(J)=FX(V(1,J))
K=NV
4
 YH=Y(1)
YL=Y(1)
NH=1
NL=1
DO 5 J=2,NV
IF(Y(J).GT.YH) THEN
YH=Y(J)
NH=J
ELSEIF(Y(J).LT.YL) THEN
YL=Y(J)
NL=J
ENDIF
5
 CONTINUE
YB=Y(1)
DO 6 J=2,NV
6
 YB=YB+Y(J)
YB=YB/NV
D=0.0
DO 7 J=1,NV
7
 D=D+(Y(J)-YB)**2
SDA=SQRT(D/NV)
IF((K.GE.M).OR.(SDA.LE.SD)) THEN
SD=SDA
M=K
YL=Y(NL)
DO 8 I=1,N
8
 X(I)=V(I,NL)
 RETURN END IF
DO 10 I=1,N XC(I)=0.0
DO 9 J=1,NV
9
 IF(J.NE.NH) XC(I)=XC(I)+V(I,J)
10
 XC(I)=XC(I) /N
DO 11 I=1,N
11
 X(I)=XC(I)+XC(I)-V(I,NH)
K=K+1
YR=FX(X)
IF(YR.LT.YL) THEN
DO 12 I=1,N
12
 XX(I)=X(I)+X(I)-XC(I)
K=K+1
YE=FX(XX)
IF(YE.LT.YR) THEN
Y(NH)=YE
DO 13 I=1,N
13
 V(I,NH)=XX(I)
ELSE
Y(NH)=YR
DO 14 I=1,N
14
 V(I,NH)=X(I)
END IF
GOTO 4
ENDIF
Y2=Y(NL)
DO 15 J=1,NV
15
 IF((J.NE.NL).AND.(J.NE.NH).AND.(Y(J).GT.Y2))IF(YR.LT.YH) THEN
Y(NH)=YR
DO 16 I=1,N
16
 V(I,NH)=X(I)
IF(YR.LT.Y2) GO TO 4
ENDIF
DO 17 I=1,N
17 XX(I)=0.5*(V(I,NH)+XC(I))
K=K+1
YC=FX(XX)
IF(YC.LT.YH) THEN
Y(NH)=YC
DO 18 I=1,N
18
 V(I,NH)=XX(I)
ELSE
DO 20 J=1,NV
DO 19 I=1,N
19
 V(I,J)=0.5*(V(I,J)+V(I,NL))
20
 IF(J.NE.NL) Y(J)=FX(V(1,J))
K=K+N
ENDIF
GO TO 4
END
B.2 NUMERICAL LINEARIZATION SUBROUTINE
Subroutine JACOB will calculate Jacobian matrices for the set of nonlinear state
equations contained in the subroutine F (specified as an argument of JACOB). Sub-
routine F(TIME, X, XD) should contain “CONTROLS” and “OUTPUT” common
blocks as used in the text. The argument FN is a double-precision function used to
determine an approximation to each partial derivative that is required.
To calculate the A, B, C, D matrices the main program should be designed to call
JACOB four times, with FN replaced in turn by each of the partial derivative functions
FDX, FDU, YDX, and YDU (given below). The partial derivative functions must be
declared “EXTERNAL” in the main program. The vectors X and XD are, respec-
tively, the state vector and its derivative. The vector V must contain the equilibrium
condition and should be replaced by X or U, respectively, depending on whether the
partial derivatives with respect to X or U are being calculated. The array IO is used to
specify the set of integers corresponding to the rows of the Jacobian matrix, and JO
is used to specify the set corresponding to the columns. NR and NC are, respectively,
the number of rows and the number of columns in the Jacobian matrix and the linear
array ABC contains the columns of the Jacobian matrix, stacked one after the other.
The linearization algorithm chooses smaller and smaller perturbations in the inde-
pendent variable and compares three successive approximations to the particular par-
tial derivative. If these approximations agree within a certain tolerance, then the size
of the perturbation is reduced to determine if an even smaller tolerance can be satis-
fied. The algorithm terminates successfully when a tolerance TOLMIN is reached or
if a tolerance of at least OKTOL can be achieved. If the algorithm does not terminate
successfully, then the successive approximations are displayed and the user is asked
to decide on the value of the partial derivative.
SUBROUTINE JACOB (FN,F,X,XD,V,IO,JO,ABC,NR,NC)
DIMENSION X(*),XD(*),V(*),IO(*),JO(*),ABC(*)
EXTERNAL FN,F
LOGICAL FLAG, DIAGS
CHARACTER*1 ANS
REAL*8 FN,TDV
DATA DEL,DMIN,TOLMIN,OKTOL /.01, .5, 3.3E-5, 8.1E-4/
C
DIAGS= .TRUE.
PRINT ’(1X,A,$)’, ’DIAGNOSTICS ? (Y/N, "/"=N) ’
READ(*,’(A)’) ANS
IF (ANS .EQ.’/’.0R. ANS .EQ. ’N’ .0R. ANS .EQ. ’n’)DIAGS=.FALSE.
IJ= 1
DO 40 J=1,NC
DV= AMAX1( ABS( DEL*V(JO(J)) ), DMIN )
DO 40 I=1,NR
FLAG= .FALSE.
1
 TOL= 0.1
OLTOL= TOL
TDV= DBLE( DV )
A2= 0.0
A1= 0.0
A0= 0.0
B1= 0.0
B0= 0.0
D1= 0.0
D0= 0.0
IF (DIAGS .OR. FLAG) WRITE(*,’(/1X,A8,I2,A1,I2,11X,A12,8X,A5)’)
& ’Element ’,I,’,’,J, ’perturbation’,’slope’
DO 20 K= 1,18 ! iterations on TDV
A2= A1
A1= A0
B1= B0
D1= D0
A0= FN(F,XD,X,IO(I),JO(J),TDV)
Bφ= AMIN1( ABS(A0), ABS(A1) )
D0= ABS ( A0 - A1 )
IF (DIAGS .OR. FLAG) WRITE(*,’(20X,1P2E17.6)’) TDV,A0
IF(K .LE. 2) GO TO 20
IF (A0 .EQ. A1 .AND. A1 .EQ. A2) THEN
ANS2= A1
GO TO 30
END IF
IF (A0 .EQ. 0.0) GO TO 25
10
 IF( D0 .LE. TOL*B0 .AND. D1 .LE. TOL*B1) THEN
ANS2= A1
OLTOL= TOL
IF(DIAGS .0R. FLAG) WRITE(*,’(1X,A9,F8.7)’) ’MET TOL= ’,TOL
IF (TOL .LE. TOLMIN) THEN
GO TO 30
ELSE
TOL= 0.2*TOL
GO TO 10
END IF
END IF
20
 TDV= 0.6D0*TDV
25
 IF (OLTOL .LE. OKTOL) THEN
GO TO 30
ELSE IF (.NOT. FLAG) THEN
WRITE(*,’(/1X,A)’)’NO CONVERGENCE *****’
FLAG= .TRUE.
GO TO 1 ELSE
21
 WRITE(*,’(1X,A,$)’) ’Enter estimate : ’
READ(*,*,ERR=21) ANS2
FLAG= .FALSE.
GO TO 30
END IF
30
 ABC(IJ)= ANS2
IF (DIAGS) THEN
PRINT ’(27X,A5,1PE13.6)’,’Ans= ’,ANS2
PAUSE ’Press "enter"’
END IF
40
 IJ= IJ+1
RETURN END
DOUBLE PRECISION FUNCTION FDX(F,XD,X,I,J,DDX)
REAL*4 XD(*), X(*)
DOUBLE PRECISION T, DDX, XD1, XD2
EXTERNAL F
TIME= 0.0
T
 = DBLE( X(J) )
X(J)= SNGL( T - DDX )
CALL F(TIME,X,XD)
XD1 = DBLE( XD(I) )
X(J)= SNGL( T + DDX )
CALL F(TIME,X,XD)
XD2 = DBLE( XD(I) )
FDX = (XD2-XD1)/(DDX+DDX)
X(J)= SNGL( T )
RETURN
END
DOUBLE PRECISION FUNCTION FDU(F,XD,X,I,J,DDU)
PARAMETER (NIN=10)
REAL*4 XD(*), X(*)
COMMON/CONTROLS/U(NIN)
DOUBLE PRECISION T, DDU, XD1, XD2
EXTERNAL F
TIME= 0.0
T
 = DBLE( U(J) )
U(J)= SNGL( T - DDU )
CALL F(TIME,X,XD)
XD1 = DBLE( XD(I) )
U(J)= SNGL( T + DDU )
CALL F(TIME,X,XD)
XD2 = DBLE( XD(I) )
FDU = (XD2-XD1)/(DDU+DDU)
U(J)= SNGL( T )
RETURN
END
DOUBLE PRECISION FUNCTION YDX(F,XD,X,I,J,DDX)
PARAMETER (NOP=20)
REAL*4 XD(*), X(*)
COMMON/OUTPUT/Y/(NOP)
DOUBLE PRECISION T, DDX, YD1, YD2
EXTERNAL F
TIME= 0.0
T
 = DBLE( X(J) )
X(J)= SNGL( T - DDX )
CALL F(TIME,X,XD)
YD1 = DBLE( Y(I) )
X(J)= SNGL( T + DDX )
CALL F(TIME,X,XD)
YD2 = DBLE( Y(I) )
YDX = (YD2-YD1)/(DDX+DDX)
X(J)= SNGL(T)
RETURN
END
DOUBLE PRECISION FUNCTION YDU(F,XD,X,I,J,DDU)
PARAMETER (NIN=10, NOP=20)
REAL*4 XD(*), X(*)
COMMON/CONTROLS/U(NIN)
COMMON/OUTPUT/Y(NOP)
DOUBLE PRECISION T, DDU, YD1, YD2
EXTERNAL F
TIME= 0.0
T
 = DBLE( U(J) )
U(J)= SNGL( T - DDU )
CALL F(TIME,X,XD)
YD1 = DBLE( Y(I) )
U(J)= SNGL( T + DDU )
CALL F(TIME,X,XD)
YD2 = DBLE( Y(I) )
YDU = (YD2-YD1)/(DDU+DDU)
U(J)= SNGL(T)
RETURN
END
APPENDIX B
 731
B.3 RUNGE-KUTTA INTEGRATION
This subroutine implements “Runge’s fourth-order rule” as described in Chapter 3.
Its arguments are the subroutine F containing the nonlinear state equations, the cur-
rent time TT, the integration time step DT, the state and state derivative vectors XX
and XD, and the number of state variables NX. Subroutine F should be declared
EXTERNAL in the main program unit.
SUBROUTINE RK4(F,TT,DT,XX,XD,NX)
PARAMETER (NN=30) ! same as main prog.
REAL*4 XX(*),XD(*),X(NN),XA(NN)
CALL F(TT,XX,XD)
DO 1 M=1,NX
XA (M)=XD(M)*DT
1
 X(M)=XX(M)+0.5*XA(M)
T=TT+0.5*DT
CALL F(T,X,XD)
DO 2 M=1,NX
Q=XD(M)*DT
X(M)=XX(M)+0.5*Q
2
 XA(M)=XA(M)+Q+Q
CALL F(T,X,XD)
DO 3 M=1,NX
Q=XD(M)*DT
X(M)=XX(M)+Q
3
 XA(M)=XA(M)+Q+Q
TT=TT+DT
CALL F(TT,X,XD)
DO 4 M=1,NX
4
 XX(M)=XX(M)+(XA(M)+XD(M)*DT)/6.0
RETURN
END


B.4 OUTPUT FEEDBACK DESIGN
Output feedback design is not an easy problem. Finding the optimal output feedback
gains to minimize a quadratic performance index (PI),
∞
1
J =
 (xT Qx + uT Ru) dt,
 (B.7.1)
2 ∫0
involves solving coupled nonlinear matrix design equations of the form (Chapter 5)
∂H
 T T0 =
 = AT c P + PAc + C K RKC + Q
 (B.7.2)
∂S
∂H
0 =
 = Ac S + SAT c + X
 (B.7.3)
∂P
1 ∂H
 T0 =
 = RKCSC − BT PSCT ,
 (B.7.4)
2 ∂K
where
Ac = A − BKC,
 X = x(0)xT (0)
In the design of tracking systems, the equations are even worse.
We have used two general approaches to solving such equation sets. In the first,
the PI J is computed based on (B.7.2) using
1
J =
 tr(PX)
2
(B.7.5)
The Simplex routine in Section B.1 was used to minimize J. In the second approach,
a gradient algorithm (e.g., Davidon-Fletcher-Powell) was used.* There, the gradient
∂J∕∂K is computed using all three design equations (B.7.2) to (B.7.4).