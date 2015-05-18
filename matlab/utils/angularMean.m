function mu = angularMean(angles, modulo)

if nargin < 2,
  modulo = 360.0;
end

if modulo ~= pi,
  angles = angles/modulo*pi;
end

mu = atan2(mean(sin(angles)), mean(cos(angles)))/pi*modulo;

end
