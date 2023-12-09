function [FY,FX] = MF52_Combined(SlipRatio,SlipAngle,NormalLoad,Camber)
% Input - [SL, SA, FZ, IA] 
% Output - [FY, FX]
% Uses Coefficients Determined by Individual Lateral and Longitudinal
% Fitting Process

% Inputs

FZ0 = 8.000e+002;

KAPPA = SlipRatio; % Slip Ratio [-]
ALPHA = SlipAngle; % [Rad]
FZ = abs(NormalLoad); % [N]
GAMMA = Camber; % [Rad]

LFZO = 1;
FZ0PR = FZ0 * LFZO; %15, NEED LFZO NOT LFZ0 TO MATCH TIRE PROP FILE
DFZ = (FZ-FZ0PR ) ./ FZ0PR ; %14, (%30)

% Scaling Factors

LGAY    = 1.1;
LHY     =   1;
LVY     =   1;
LCY     =   1;
LEY     =   1;
LMUY    =   0.5; % Changed to Fit
LHX     =   1;
LVX     =   1;
LMUX    =   0.3;
LGX     =   1;
LCX     =   1;
LEX     =   1;
LKX     =   1;
LXAL    =   1;

% Pure Lateral Coefficients
PCY1                      = +1.757e+000;%    $typarr( 91)
PDY1                      = +2.574e+000;%    $typarr( 92)
PDY2                      = -5.027e-001;%    $typarr( 93)
PDY3                      = -5.992e-001;%    $typarr( 94)
PEY1                      = -5.379e-001;%    $typarr( 95)
PEY2                      = -1.113e+000;%    $typarr( 96)
PEY3                      = +3.180e-001;%    $typarr( 97)
PEY4                      = -5.013e+000;%    $typarr( 98)
PKY1                      = -5.785e+001;%    $typarr( 99)
PKY2                      = +1.785e+000 ;%   $typarr(100)
PKY3                      = +5.450e-001;%    $typarr(101)
PHY1                      = +0.000e+000;%    $typarr(102)
PHY2                      = +0.000e+000;%    $typarr(103)
PHY3                      = -3.408e-004;%    $typarr(104)
PVY1                      = +0.000e+000;%    $typarr(105)
PVY2                      = +0.000e+000;%    $typarr(106)
PVY3                      = -2.652e+000;%    $typarr(107)
PVY4                      = -7.016e-001;%    $typarr(108)
RBY1                      = +2.833e+001;%    $typarr(109)
RBY2                      = +1.190e+001;%    $typarr(110)
RBY3                      = -1.243e-002;%    $typarr(111)
RCY1                      = +9.317e-001;%    $typarr(112)
REY1                      = -3.982e-004;%    $typarr(113)
REY2                      = +3.077e-001;%    $typarr(114)
RHY1                      = +0.000e+000;%    $typarr(115)
RHY2                      = +0.000e+000;%    $typarr(116)
RVY1                      = +0.000e+000;%    $typarr(117)
RVY2                      = +0.000e+000;%    $typarr(118)
RVY3                      = +0.000e+000;%    $typarr(119)
RVY4                      = +0.000e+000;%    $typarr(120)
RVY5                      = +0.000e+000;%    $typarr(121)
RVY6                      = +0.000e+000 ;%   $typarr(122)
% Pure Longitudinal Coefficients
PCX1                      = +1.616e+000 ;  
PDX1                      = +2.749e+000 ;  
PDX2                      = -2.010e-001 ;  
PDX3                      = +1.223e+001 ; %  $typarr( 60)
PEX1                      = +7.202e-001 ; %    $typarr( 64)
PEX2                      = -9.124e-002 ; %    $typarr( 65)
PEX3                      = +2.430e-002; %     $typarr( 66)
PEX4                      = -7.010e-002; %     $typarr( 67)
PKX1                      = +7.530e+001; %     $typarr( 68)
PKX2                      = -2.025e+001; %     $typarr( 69)
PKX3                      = +4.110e-001; %     $typarr( 70)
PHX1                      = +0.000e+000; %     $typarr( 71)
PHX2                      = +0.000e+000; %     $typarr( 72)
PVX1                      = +0.000e+000; %     $typarr( 73)
PVX2                      = +0.000e+000; %     $typarr( 74)
RBX1                      = +1.391e+001; %     $typarr( 75)
RBX2                      = +1.485e+001; %     
RCX1                      = +7.595e-001; %     
REX1                      = -7.759e-001  ; % 
REX2                      = +5.009e-001    ; % 
RHX1                      = +0.000e+000    ; % 

%% Lateral Code

% Pure Lateral Force
GAMMAY = GAMMA.*LGAY;
SHY = (PHY1+ PHY2.*DFZ).*LHY + PHY3.*GAMMAY;
SVY = 0; %FZ.*((PVY1 + PVY2.*DFZ).*LVY + (PVY3 + PVY4.*DFZ)).*LMUY;
ALPHAY = ALPHA + SHY;
CY = PCY1.*LCY;
MUY = (PDY1 + PDY2.*DFZ).*(1 - PDY3.*(GAMMAY.^2)).*LMUY;
DY = MUY.*FZ;
EY = (PEY1 + PEY2.*DFZ).*(1 - (PEY3 + PEY4.*GAMMAY).*sign(ALPHAY)).*LEY;

KY0 = PKY1.*FZ0.*sin( 2.*atan(FZ./(PKY2.*FZ0PR))); % Cornering Stiffness
KVY0 = PHY3.*KY0 + FZ.*(PVY3 + PVY4.*DFZ); % Camber Stiffness
KY = KY0.*(1 - PKY3.*abs(GAMMAY));
BY = KY./(CY.*DY);
FY0 = DY.*sin(CY.*atan(BY.*ALPHAY - EY.*(BY.*ALPHAY - atan(BY.*ALPHAY)))) + SVY;
FY = FY0;

% Combined Lateral Force

BYK = RBY1.*cos(atan(RBY2.*(ALPHA - RBY3)));
CYK = RCY1;
EYK = REY1 + REY2.*DFZ;

SHYK = RHY1 + RHY2.*DFZ; % This kills 
KS = KAPPA + SHYK;
DVYK = MUY.*FZ.*(RVY1 + RVY2.*DFZ + RVY3.*GAMMA).*cos(atan(RVY4.*ALPHA));
SVYK = DVYK.*sin(RVY5.*atan(RVY6.*KAPPA));
DYK = FY0./( cos(CYK.*atan(BYK.*SHYK - EYK.*(BYK.*SHYK - atan(BYK.*SHYK)))));
GYK = ( cos(CYK.*atan( BYK.*KS - EYK.*(BYK.*KS - atan(BYK.*KS))))) ./ ( cos(CYK.*atan(BYK.*SHYK - EYK.*(BYK.*SHYK - atan(BYK.*SHYK)))));

FY_C = GYK.*FY + SVYK;

% FY = (DYK.*cos(CYK.*atan( BYK.*KS - EYK.*(BYK.*KS - atan(BYK.*KS))))) + SVYK;

FY_Combined = FY_C; % Divide by 1.3 for Closer Fudge Factor

FY = FY_Combined;

%% Longitudinal Code

% Pure Longitudinal Code
SHX = (PHX1 + PHX2.*DFZ).*LHX;
SVX = FZ.*(PVX1 + PVX2.*DFZ).*LVX.*LMUX;
KAPPAX = KAPPA + SHX;
GAMMAX = GAMMA.*LGX;
CX = PCX1.*LCX;
MUX = (PDX1 + PDX2.*DFZ).*(1 - PDX3.*(GAMMA.^2)).*LMUX;
DX = MUX.*FZ;
EX = (PEX1 + PEX2.*DFZ + PEX3.*(DFZ.^2)).*(1 - PEX4.*sign(KAPPAX)).*LEX;

KX = FZ.*(PKX1 + PKX2.*DFZ).*exp(PKX3.*DFZ).*LKX; % Longitudinal Slip Stiffness
BX = KX./(CX.*DX);
FX0 = DX.*sin( (CX.*atan(BX.*KAPPAX - EX.*(BX.*KAPPAX - atan(BX.*KAPPAX)))) + SVX);

% Combined Longitudinal Equations

SHXAL = RHX1;
CXAL = RCX1;
BXAL = RBX1.*cos( atan(RBX2.*KAPPA)).*LXAL; % cos term will always be positive regardless of slip ratio direction
ALPHAS = ALPHA + SHXAL;
EXAL = REX1 + REX2.*DFZ;

GXAL = ( cos(CXAL.*atan(BXAL.*ALPHAS - EXAL.*(BXAL.*ALPHAS - atan(BXAL.*ALPHAS))))) ./ ( cos(CXAL.*atan(BXAL.*SHXAL - EXAL.*(BXAL.*SHXAL - atan(BXAL.*SHXAL)))));

FX_C = abs(GXAL).*FX0;

FX = FX_C;
end

