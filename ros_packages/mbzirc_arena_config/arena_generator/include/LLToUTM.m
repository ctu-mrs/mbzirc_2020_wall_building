function [x, y] = LLToUTM(lat, lon)

RADIANS_PER_DEGREE = pi / 180.0;
DEGREES_PER_RADIAN = 180.0 / pi;

% WGS84 Parameters
WGS84_A  = 6378137.0;         % major axis
WGS84_B  = 6356752.31424518;  % minor axis
WGS84_F  = 0.0033528107;      % ellipsoid flattening
WGS84_E  = 0.0818191908;      % first eccentricity
WGS84_EP = 0.0820944379;      % second eccentricity

% UTM Parameters
UTM_K0   = 0.9996;                   % scale factor
UTM_FE   = 500000.0;                 % false easting
UTM_FN_N = 0.0;                      % false northing on north hemisphere
UTM_FN_S = 10000000.0;               % false northing on south hemisphere
UTM_E2   = (WGS84_E * WGS84_E);      % e^2
UTM_E4   = (UTM_E2 * UTM_E2);        % e^4
UTM_E6   = (UTM_E4 * UTM_E2);        % e^6
UTM_EP2  = (UTM_E2 / (1 - UTM_E2));  % e'^2


% constants
m0 = (1 - UTM_E2 / 4 - 3 * UTM_E4 / 64 - 5 * UTM_E6 / 256);
m1 = -(3 * UTM_E2 / 8 + 3 * UTM_E4 / 32 + 45 * UTM_E6 / 1024);
m2 = (15 * UTM_E4 / 256 + 45 * UTM_E6 / 1024);
m3 = -(35 * UTM_E6 / 3072);

% compute the central meridian
if (lon >= 0.0)
  cm = floor(lon) - mod(floor(lon), 6) + 3;
else
  cm = floor(lon) - mod(floor(lon), 6) - 3;
end

% convert degrees into radians
rlat  = lat * RADIANS_PER_DEGREE;
rlon  = lon * RADIANS_PER_DEGREE;
rlon0 = cm * RADIANS_PER_DEGREE;

% compute trigonometric functions
slat = sin(rlat);
clat = cos(rlat);
tlat = tan(rlat);

% decide the false northing at origin
if (lat > 0)
  fn =  UTM_FN_N;
else
  fn = UTM_FN_S;
end

T = tlat * tlat;
C = UTM_EP2 * clat * clat;
A = (rlon - rlon0) * clat;
M = WGS84_A * (m0 * rlat + m1 * sin(2 * rlat) + m2 * sin(4 * rlat) + m3 * sin(6 * rlat));
V = WGS84_A / sqrt(1 - UTM_E2 * slat * slat);

% compute the easting-northing coordinates
x = UTM_FE + UTM_K0 * V * (A + (1 - T + C) * A^3 / 6 + (5 - 18 * T + T * T + 72 * C - 58 * UTM_EP2) * A^5 / 120);
y = fn + UTM_K0 * (M + V * tlat * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A^4 / 24 + ((61 - 58 * T + T * T + 600 * C - 330 * UTM_EP2) * A^6 / 720)));

end
