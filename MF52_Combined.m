function [FY,FX] = MF52_Combined(SlipRatio,SlipAngle,NormalLoad,Camber)
% Input - [SL, SA, FZ, IA] 
% Output - [FY, FX]
% Uses Coefficients Determined by Individual Lateral and Longitudinal
% Fitting Process

% Inputs

FZ0 = 1100;

KAPPA = SlipRatio; % Slip Ratio [-]
ALPHA = SlipAngle; % degrees to [Rad]
FZ = (NormalLoad); % [N]
GAMMA = Camber*pi/180; % degrees to [Rad]

LFZO = 1.2;
FZ0PR = FZ0 * LFZO; %15, NEED LFZO NOT LFZ0 TO MATCH TIRE PROP FILE
DFZ = (FZ-FZ0PR ) ./ FZ0PR ; %14, (%30)

% Scaling Factors

LGAY    =   1;
LHY     =   1;
LVY     =   1;
LCY     =   1;
LEY     =   1;
LHX     =   0.4;
LVX     =   1;
LGX     =   1;
LCX     =   1;
LEX     =   1;
LXAL    =   1;

LKY     =   1; % 1
LKX     =   1; %  0.7
LMUY    =   0.38; % 0.38 Changed to Fit
LMUX    =   0.35; % 0.25 Changed to Fit

% Longitudinal Coefficients
PCX1=1.2602;
PDX1=2.354;
PDX2=-0.015401;
PDX3=-0.76992;
PEX1=-1.0845;
PEX2=2.3203;
PEX3=3.2136;
PEX4=-1.7027;
PKX1=39.334;
PKX2=-0.37146;
PKX3=0.37752;
PHX1=0.025058;
PHX2=-0.038843;
PVX1=-0.00045953;
PVX2=0.0013401;
% Combined Longitudinal Coefficients
RBX1=7.4574;
RBX2=-8.8044;
RCX1=1.5974;
REX1=0.22918;
REX2=-0.5217;
RHX1=0;

% Lateral Coefficients
PCY1=1.4;
PDY1=2.4;
PDY2=-0.4507889;
PDY3=20;
PEY1=0.01;
PEY2=0.05;
PEY3=10;
PEY4=0;
PKY1=-27.3678;
PKY2=1.242483;
PKY3=3;
PHY1=-0.00002845241;
PHY2=-0.0000329537;
PHY3=0.1416031;
PVY1=0;
PVY2=-0.009009;
PVY3=-0.5;
PVY4=-1;
% Combined Lateral Coefficients
RBY1=26.3099;
RBY2=20.3304;
RBY3=-0.015204;
RCY1=0.96889;
REY1=0.53522;
REY2=0.69602;
RHY1=0;
RHY2=0;
RVY1=0;
RVY2=0;
RVY3=0;
RVY4=0;
RVY5=0;
RVY6=0;


%% Lateral Force

GAMMAY = GAMMA.*LGAY;
SHY = (PHY1+ PHY2.*DFZ).*LHY + PHY3.*GAMMAY;
SVY = FZ.*((PVY1 + PVY2.*DFZ).*LVY + (PVY3 + PVY4.*DFZ)).*LMUY;
ALPHAY = ALPHA + SHY;
CY = PCY1.*LCY;
MUY = (PDY1 + PDY2.*DFZ).*(1 - PDY3.*(GAMMAY.^2)).*LMUY;
DY = MUY.*FZ;
EY = (PEY1 + PEY2.*DFZ).*(1 - (PEY3 + PEY4.*GAMMAY).*sign(ALPHAY)).*LEY;

KY0 = PKY1.*FZ0.*sin( 2.*atan(FZ./(PKY2.*FZ0PR))).*LKY; % Cornering Stiffness
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

FY = FY_C;

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
FX_C = GXAL.*FX0;
FX = FX_C;
end

