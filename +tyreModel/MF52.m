function [FY,FX] = MF52(SlipRatio,SlipAngle,NormalLoad,Camber, carData)
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

LFZO = carData.Tyre.LFZO;
FZ0PR = FZ0 * LFZO; %15, NEED LFZO NOT LFZ0 TO MATCH TIRE PROP FILE
DFZ = (FZ-FZ0PR ) ./ FZ0PR ; %14, (%30)

% Scaling Factors

LGAY    =   carData.Tyre.LGAY;
LHY     =   carData.Tyre.LHY;
LVY     =   carData.Tyre.LVY;
LCY     =   carData.Tyre.LCY;
LEY     =   carData.Tyre.LEY;
LHX     =   carData.Tyre.LHX;
LVX     =   carData.Tyre.LVX;
LGX     =   carData.Tyre.LGX;
LCX     =   carData.Tyre.LCX;
LEX     =   carData.Tyre.LEX;
LXAL    =   carData.Tyre.LXAL;

LKY     =   carData.Tyre.LKY; % 1
LKX     =   carData.Tyre.LKX; %  0.7
LMUY    =   carData.Tyre.LMUY; % 0.38 Changed to Fit
LMUX    =   carData.Tyre.LMUX; % 0.25 Changed to Fit

% Longitudinal Coefficients
PCX1    =   carData.Tyre.PCX1;
PDX1    =   carData.Tyre.PDX1;
PDX2    =   carData.Tyre.PDX2;
PDX3    =   carData.Tyre.PDX3;
PEX1    =   carData.Tyre.PEX1;
PEX2    =   carData.Tyre.PEX2;
PEX3    =   carData.Tyre.PEX3;
PEX4    =   carData.Tyre.PEX4;
PKX1    =   carData.Tyre.PKX1;
PKX2    =   carData.Tyre.PKX2;
PKX3    =   carData.Tyre.PKX3;
PHX1    =   carData.Tyre.PHX1;
PHX2    =   carData.Tyre.PHX2;
PVX1    =   carData.Tyre.PVX1;
PVX2    =   carData.Tyre.PVX2;

% Combined Longitudinal Coefficients
RBX1    =   carData.Tyre.RBX1;
RBX2    =   carData.Tyre.RBX2;
RCX1    =   carData.Tyre.RCX1;
REX1    =   carData.Tyre.REX1;
REX2    =   carData.Tyre.REX2;
RHX1    =   carData.Tyre.RHX1;

% Lateral Coefficients
PCY1    =   carData.Tyre.PCY1;
PDY1    =   carData.Tyre.PDY1;
PDY2    =   carData.Tyre.PDY2;
PDY3    =   carData.Tyre.PDY3;
PEY1    =   carData.Tyre.PEY1;
PEY2    =   carData.Tyre.PEY2;
PEY3    =   carData.Tyre.PEY3;
PEY4    =   carData.Tyre.PEY4;
PKY1    =   carData.Tyre.PKY1;
PKY2    =   carData.Tyre.PKY2;
PKY3    =   carData.Tyre.PKY3;
PHY1    =   carData.Tyre.PHY1;
PHY2    =   carData.Tyre.PHY2;
PHY3    =   carData.Tyre.PHY3;
PVY1    =   carData.Tyre.PVY1;
PVY2    =   carData.Tyre.PVY2;
PVY3    =   carData.Tyre.PVY3;
PVY4    =   carData.Tyre.PVY4;

% Combined Lateral Coefficients
RBY1    =   carData.Tyre.RBY1;
RBY2    =   carData.Tyre.RBY2;
RBY3    =   carData.Tyre.RBY3;
RCY1    =   carData.Tyre.RCY1;
REY1    =   carData.Tyre.REY1;
REY2    =   carData.Tyre.REY2;
RHY1    =   carData.Tyre.RHY1;
RHY2    =   carData.Tyre.RHY2;
RVY1    =   carData.Tyre.RVY1;
RVY2    =   carData.Tyre.RVY2;
RVY3    =   carData.Tyre.RVY3;
RVY4    =   carData.Tyre.RVY4;
RVY5    =   carData.Tyre.RVY5;
RVY6    =   carData.Tyre.RVY6;


% Lateral Force

GAMMAY = GAMMA.*LGAY;
SHY = (PHY1+ PHY2.*DFZ).*LHY + PHY3.*GAMMAY;
SVY = 0; % FZ.*((PVY1 + PVY2.*DFZ).*LVY + (PVY3 + PVY4.*DFZ)).*LMUY;
ALPHAY = ALPHA + SHY;
CY = PCY1.*LCY;
MUY = (PDY1 + PDY2.*DFZ).*(1 - PDY3.*(GAMMAY.^2)).*LMUY;
DY = MUY.*FZ;
EY = (PEY1 + PEY2.*DFZ).*(1 - (PEY3 + PEY4.*GAMMAY).*sign(ALPHAY)).*LEY;
KY0 = PKY1.*FZ0.*sin( 2.*atan(FZ./(PKY2.*FZ0PR))).*LKY; % Cornering Stiffness
KVY0 = PHY3.*KY0 + FZ.*(PVY3 + PVY4.*DFZ); % Camber Stiffness
KY = KY0.*(1 - PKY3.*abs(GAMMAY));
BY = KY./(CY.*DY);
FY0 = DY.*sin(CY.*atan(BY.*ALPHAY - EY.*(BY.*ALPHAY - atan(BY.*ALPHAY)))) + SVY; % Pure Slip Lateral Force

BYK = RBY1.*cos(atan(RBY2.*(ALPHA - RBY3)));
CYK = RCY1;
EYK = REY1 + REY2.*DFZ;
SHYK = RHY1 + RHY2.*DFZ; % This kills 
KS = KAPPA + SHYK;
DVYK = MUY.*FZ.*(RVY1 + RVY2.*DFZ + RVY3.*GAMMA).*cos(atan(RVY4.*ALPHA));
SVYK = DVYK.*sin(RVY5.*atan(RVY6.*KAPPA));
DYK = FY0./( cos(CYK.*atan(BYK.*SHYK - EYK.*(BYK.*SHYK - atan(BYK.*SHYK)))));
GYK = ( cos(CYK.*atan( BYK.*KS - EYK.*(BYK.*KS - atan(BYK.*KS))))) ./ ( cos(CYK.*atan(BYK.*SHYK - EYK.*(BYK.*SHYK - atan(BYK.*SHYK)))));
FY = GYK.*FY0 + SVYK; % Combined Slip Lateral Force

% Longitudinal Force

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
FX0 = DX.*sin( (CX.*atan(BX.*KAPPAX - EX.*(BX.*KAPPAX - atan(BX.*KAPPAX)))) + SVX); % Pure Slip Longitudinal Force

SHXAL = RHX1;
CXAL = RCX1;
BXAL = RBX1.*cos( atan(RBX2.*KAPPA)).*LXAL; % cos term will always be positive regardless of slip ratio direction
ALPHAS = ALPHA + SHXAL;
EXAL = REX1 + REX2.*DFZ;
GXAL = ( cos(CXAL.*atan(BXAL.*ALPHAS - EXAL.*(BXAL.*ALPHAS - atan(BXAL.*ALPHAS))))) ./ ( cos(CXAL.*atan(BXAL.*SHXAL - EXAL.*(BXAL.*SHXAL - atan(BXAL.*SHXAL)))));
FX = GXAL.*FX0; % Combined Slip Slip Longitudinal Force

end

