function tag = genFTag2MarkerFromPhases(phases, tagWidthPx, marginSliceCount, oversamplePct)
% Generate FTag2 marker image given payload matrix of phases
%
% Variables:
% - S (numSlices): number of signal slices in tag; == size(phases, 1)
% - M (marginSliceCount): extra width, in slice units, for border margin
% - W (tagWidthPx): width of tag in pixels
% - O (oversamplePct): sinusoid oversampling percentage
% - c: horizontal pixel index
% - r: vertical pixel index
% - t: near-normalized position in sine signal, e.g. 0-S*O == left edge,
%      S+S*O == right edge
%
% Remarks:
% - each pixel represents the CENTER of an image patch
% - the tag has a width of EXACTLY W pixels
% - thus the left border of the tag is located at (c=1-1/2/W pixels)
% - and the right border of the tag is located at (c=W+1/2/W pixels)
% - the borders have mappings: t=0-S*O-M for left, t=S+S*O+M for right
% - the same logic for horizontal t <-> c relationship also applies for
%   vertical t <-> s conversions, except that O == 0
%
%
% Derivations of relationship between c and t:
% > syms B S M W O c t alpha beta
% > expr = alpha*c/W*(S+2*M)+beta - t/S; % == 0
% > expr1 = subs(subs(expr, c, (1-1/2/W)), t, 0-S*O-M); % == 0
% > expr2 = subs(subs(expr, c, (W+1/2/W)), t, S+S*O+M); % == 0
% > [alphaSln, betaSln] = solve(expr1==0, expr2==0, alpha, beta);
% > exprSln = simplify(subs(subs(expr, alpha, alphaSln), beta, betaSln));
%
% Simplified validation: (removing a 1/S factor from exprSln)
% > exprS = c*(W)*(S+2*S*O+2*M)/(W^2-W+1) + (1-2*W)*(S+2*S*O+2*M)/2/(W^2-W+1) - (t+S*O+M); % == 0
% > assert(simplify(exprS/S - exprSln) == 0);
% > assert(simplify(subs(subs(exprSln, c, (1-1/2/W)), t, 0-S*O-M)) == 0);
% > assert(simplify(subs(subs(exprSln, c, (W+1/2/W)), t, S+S*O+M)) == 0);
%
% Final relationships:
% t = c*W*(S+2*S*O+2*M)/(W^2-W+1) + (1-2*W)*(S+2*S*O+2*M)/2/(W^2-W+1) - (S*O+M)
% c = t*(W^2-W+1)/W/(S+2*S*O+2*M) + (M+S*O)*(W^2-W+1)/W/(S+2*S*O+2*M) - (1-2*W)/(2*W)
% > assert( simplify(subs(exprSln, t, (c*W*(S+2*O*S+2*M)/(W^2-W+1) + (1-2*W)*(S+2*O*S+2*M)/2/(W^2-W+1) - (O*S+M)))) == 0 );
% > assert( simplify(subs(exprSln, c, (t*(W^2-W+1)/W/(S+2*S*O+2*M) + (M+S*O)*(W^2-W+1)/W/(S+2*S*O+2*M) - (1-2*W)/(2*W)))) == 0 );

% t = c*W*(S+2*S*O+2*M)/(W^2-W+1) - (2*M*W - S + 2*S*W + 2*M*W^2 + 2*S*O*W + 2*S*O*W^2)/(2*(W^2-W+1))
% c = t*(W^2-W+1)/W/(S+2*O*S+2*M) + (2*M*W - S + 2*S*W + 2*M*W^2 + 2*S*O*W + 2*S*O*W^2)/(2*W*(S+2*S*O+2*M))
% > assert( simplify(subs(exprSln, t, (c*W*(S+2*S*O+2*M)/(W^2-W+1) - (2*M*W - S + 2*S*W + 2*M*W^2 + 2*S*O*W + 2*S*O*W^2)/(2*(W^2-W+1))))) == 0 );
% > assert( simplify(subs(exprSln, c, (t*(W^2-W+1)/W/(S+2*O*S+2*M) + (2*M*W - S + 2*S*W + 2*M*W^2 + 2*S*O*W + 2*S*O*W^2)/(2*W*(S+2*S*O+2*M))))) == 0 );

if nargin < 3,
  marginSliceCount = 1;
end
if nargin < 4,
  %oversamplePct = 0;
  oversamplePct = 0.05;
  %oversamplePct = 0.1;
end
M = marginSliceCount;
S = size(phases, 1);
F = size(phases, 2);
W = tagWidthPx;
O = oversamplePct;

tExprGain = W*(S+2*S*O+2*M)/(W^2-W+1);
tExprBias = -(2*M*W - S + 2*S*W + 2*M*W^2 + 2*S*O*W + 2*S*O*W^2)/(2*(W^2-W+1));
cExprGain = (W^2-W+1)/W/(S+2*O*S+2*M);
cExprBias = (2*M*W - S + 2*S*W + 2*M*W^2 + 2*S*O*W + 2*S*O*W^2)/(2*W*(S+2*S*O+2*M));

%tExprGainNoO = W*(S+2*M)/(W^2-W+1);
%tExprBiasNoO = -(2*M*W - S + 2*S*W + 2*M*W^2)/(2*(W^2-W+1));
cExprGainNoO = (W^2-W+1)/W/(S+2*M);
cExprBiasNoO = (2*M*W - S + 2*S*W + 2*M*W^2)/(2*W*(S+2*M));

tag = zeros(W, W, 'uint8'); % Start with black canvas

cLeft = (0-S*O)*cExprGain + cExprBias;
cRight = (S+S*O)*cExprGain + cExprBias;
cRange = ceil(cLeft):floor(cRight);
for s=1:S, % for each slice
  % Determine range of rows and columns applicable to current slice
  rTop = (s-1+0)*cExprGainNoO + cExprBiasNoO;
  rBot = (s-1+1)*cExprGainNoO + cExprBiasNoO;
  rRange = ceil(rTop):floor(rBot);
  
  % Generate linear combination of sines
  tRange = (cRange*tExprGain + tExprBias)/S;
  if any(tRange < (0-S*O)/S) || any(tRange > (S+S*O)/S),
    error('genFTag2MarkerFromPhases:OutOfBounds', 'sine signal position is out of bounds');
  end
  ray = zeros(1, length(cRange));
  for f=1:F,
    ray = ray + cosd(360*f*tRange + phases(s, f));
  end
  rayMin = min(ray);
  rayMax = max(ray);
  rayPx = uint8(round((ray-rayMin)/(rayMax-rayMin)*255));
    
  % Map ray onto tag region
  tag(rRange, cRange) = repmat(rayPx, length(rRange), 1);
end

end
