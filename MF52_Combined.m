function [FY,FX] = MF52_Combined(SlipRatio,SlipAngle,NormalLoad,Camber)
% Input - [SL, SA, FZ, IA] 
% Output - [FY, FX]
% Uses Coefficients Determined by Individual Lateral and Longitudinal
% Fitting Process

% Inputs

FZ0 = 4000;

KAPPA = SlipRatio; % Slip Ratio [-]
ALPHA = SlipAngle; % degrees to [Rad]
FZ = abs(NormalLoad); % [N]
GAMMA = Camber*pi/180; % degrees to [Rad]

LFZO = 1;
FZ0PR = FZ0 * LFZO; %15, NEED LFZO NOT LFZ0 TO MATCH TIRE PROP FILE
DFZ = (FZ-FZ0PR ) ./ FZ0PR ; %14, (%30)

% Scaling Factors

LGAY    =   1;
LHY     =   1;
LVY     =   1;
LCY     =   1;
LEY     =   1;
LHX     =   1;
LVX     =   1;
LGX     =   1;
LCX     =   1;
LEX     =   1;
LXAL    =   1;

LKY     =   1; % 1
LKX     =   1; %  0.7
LMUY    =   1; % 0.6; 0.38 Changed to Fit
LMUX    =   1; % 0.6; 0.25 Changed to Fit

% Longitudinal Coefficients
PCX1 = 1.1 ;  % ShapefactorCXforlongitudinalforce
PDX1 = 2.24071 ;  % LongitudinalfrictionµxatFZ0
PDX2 = -0.35157 ;  % Variationoffrictionµxwithload
PDX3 = 0 ;  % Variationoffrictionµxwithcamber
PEX1 = -0.01916 ;  % LongitudinalcurvatureEXatFZ0
PEX2 = -0.06434 ;  % VariationofcurvatureEXwithload
PEX3 = 0.04408 ;  % VariationofcurvatureEXwithload2
PEX4 = 0 ;  % FactorincurvatureEXwhiledriving
PKX1 = 52.3208 ;  % LongitudinalslipstiffnessKX/FZatFZ0
PKX2 = -26.3164 ;  % VariationofslipstiffnessKX/FZwithload
PKX3 = 0.51374 ;  % ExponentinslipstiffnessKX/FZwithload
PHX1 = 0.0017 ;  % HorizontalshiftSHXatFZ0
PHX2 = -0.00011 ;  % VariationofshiftSHXwithload
PVX1 = 0.00151 ;  % VerticalshiftSVX/FZatFZ0
PVX2 = -0.09205 ;  % VariationofshiftSVX/FZwithload

% Combined Longitudinal Coefficients
RBX1 = 13.4178 ;  % SlopefactorforcombinedFxreduction
RBX2 = -12.3543 ;  % VariationofslopeFxreductionwithsr
RCX1 = 1.00463 ;  % ShapefactorforcombinedslipFxreduction
REX1 = -0.48105 ;  % CurvaturefactorofCombinedFx
REX2 = 1.13752 ;  % CurvaturefactorofCombinedFxwithload
RHX1 = -0.00696 ;  % ShiftfactorforcombinedslipFxreduction

% Lateral Coefficients
PCY1 = 1.7 ;  % ShapefactorCYforlateralforce
PDY1 = -1.89243 ;  % LateralfrictionmyatFZ0
PDY2 = -0.59433 ;  % Variationoffrictionmywithload
PDY3 = 0 ;  % Variationoffrictionmywithcamber
PDY4 = 1 ;  % peaklateralforceshiftwithcamber
PEY1 = 0.31052 ;  % LongitudinalcurvatureEYatFZ0
PEY2 = 0.57622 ;  % VariationofcurvatureEYwithload
PEY3 = 0.24654 ;  % ZeroordercamberdependencyofcurvatureEY
PEY4 = 1.43263 ;  % VariationofcurvatureEYwithcamber
PKY1 = -30.5648 ;  % MaximumvalueofstiffnessKY/FZ0
PKY2 = 0.75621 ;  % LoadatwhichKYreachesmaximumvalue
PKY3 = 0.21874 ;  % VariationofKY/FZ0withcamber
PHY1 = -0.0062 ;  % HorizontalshiftSHYatFZ0
PHY2 = 0.00364 ;  % VariationofshiftSHYwithload
PHY3 = 0.07957 ;  % VariationofshiftSHywithcamber
PVY1 = -0.06085 ;  % VerticalshiftSVY/FZatFZ0
PVY2 = 0.05321 ;  % VariationofshiftSVY/FZwithload
PVY3 = 0 ;  % VariationofshiftSVY/FZwithcamber
PVY4 = 0.54363 ;  % VariationofshiftSVY/FZwithcamberandload

% Combined Lateral Coefficients
RBY1 = 16.9255 ;  % SlopefactorforcombinedFyreduction
RBY2 = -14.3112 ;  % VariationofslopeFyreductionwithsa
RBY3 = 0.00607 ;  % Shifttermfor'alpha'inslopeFyreduction
RCY1 = 1.22867 ;  % ShapefactorforcombinedFyreduction
REY1 = 0.72491 ;  % CurvaturefactorofCombinedFy
REY2 = 0.11982 ;  % CurvaturefactorofCombinedFywithload
RHY1 = 0.02236 ;  % ShiftfactorforcombinedFyreduction
RHY2 = 0.02722 ;  % ShiftfactorforcombinedFyreductionwithload
RVY1 = 0.06073 ;  % srinducedsideforceSVy'kappa'/my*FzatFz0
RVY2 = 0.03295 ;  % VariationofSVy'kappa'/my*Fzwithload
RVY3 = -0.63549 ;  % VariationofSVy'kappa'/my*Fzwithcamber
RVY4 = 18.4591 ;  % VariationofSVy'kappa'/my*Fzwithsa
RVY5 = 1.9 ;  % VariationofSVy'kappa'/my*Fzwithsr
RVY6 = -18.8069 ;  % VariationofSVy'kappa'/my*Fzwithatansr


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

% Combined Lateral Force

BYK = RBY1.*cos(atan(RBY2.*(ALPHA - RBY3)));
CYK = RCY1;
EYK = REY1 + REY2.*DFZ;

SHYK = RHY1 + RHY2.*DFZ; % This kills 
KS = KAPPA + SHYK;
DVYK = MUY.*FZ.*(RVY1 + RVY2.*DFZ + RVY3.*GAMMA).*cos(atan(RVY4.*ALPHA));
SVYK = DVYK.*sin(RVY5.*atan(RVY6.*KAPPA));
GYK = ( cos(CYK.*atan( BYK.*KS - EYK.*(BYK.*KS - atan(BYK.*KS))))) ./ ( cos(CYK.*atan(BYK.*SHYK - EYK.*(BYK.*SHYK - atan(BYK.*SHYK)))));

FY_C = GYK.*FY0 + SVYK;

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

