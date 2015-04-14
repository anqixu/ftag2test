function tag = genFTag2MarkerFromPhases(phases, tagWidthPx, marginSliceCount)
% Generate FTag2 marker image given payload matrix of phases
%
% Variables:
% - S (numSlices): number of signal slices in tag; == size(phases, 1)
% - M (marginSliceCount): extra width, in slice units, for border margin
% - W (tagWidthPx): width of tag in pixels
% - c: horizontal pixel index
% - r: vertical pixel index
% - t: near-normalized position in sine signal, e.g. 0 == left edge,
%      S == right edge
%
% Remarks:
% - each pixel represents the CENTER of an image patch
% - the tag has a width of EXACTLY W pixels
% - thus the left border of the tag is located at (c=1-1/2/W pixels)
% - and the right border of the tag is located at (c=W+1/2/W pixels)
% - the borders have mappings: t=-M for left, t=S+M for right
% - the same logic between t<->c also applies to t<->s
%
%
% Derivations of relationship between c and t:
% > syms B S M W c t alpha beta
% > expr = alpha*c/W*(S+2*M)+beta - t/S; % == 0
% > expr1 = subs(subs(expr, c, (1-1/2/W)), t, -M); % == 0
% > expr2 = subs(subs(expr, c, (W+1/2/W)), t, S+M); % == 0
% > [alphaSln, betaSln] = solve(expr1==0, expr2==0, alpha, beta);
% > exprSln = simplify(subs(subs(expr, alpha, alphaSln), beta, betaSln));
%
% Simplified validation:
% > exprS = c*(W)*(S+2*M)/(W^2-W+1) + (1-2*W)*(S+2*M)/2/(W^2-W+1) - (t+M); % == 0
% > exprS1 = simplify(subs(subs(exprS, c, (1-1/2/W)), t, -M)) % == 0
% > exprS2 = simplify(subs(subs(exprS, c, (W+1/2/W)), t, S+M)) % == 0
%
% Final relationships:
% t = c*W*(S+2*M)/(W^2-W+1) + (1-2*W)*(S+2*M)/2/(W^2-W+1) - M
% c = t*(W^2-W+1)/W/(S+2*M) + M*(W^2-W+1)/W/(S+2*M) - (1-2*W)/(2*W)

% t = c*W*(S+2*M)/(W^2-W+1) - (2*M*W - S + 2*S*W + 2*M*W^2)/(2*(W^2-W+1))
% c = t*(W^2-W+1)/W/(S+2*M) + (2*M*W - S + 2*S*W + 2*M*W^2)/(2*W*(2*M + S))

if nargin < 3,
  marginSliceCount = 1;
end
M = marginSliceCount;
S = size(phases, 1);
F = size(phases, 2);
W = tagWidthPx;

tExprGain = W*(S+2*M)/(W^2-W+1);
tExprBias = (1-2*W)*(S+2*M)/2/(W^2-W+1) - M;
cExprGain = (W^2-W+1)/W/(S+2*M);
cExprBias = M*(W^2-W+1)/W/(S+2*M) - (1-2*W)/(2*W);

tag = zeros(W, W, 'uint8'); % Start with black canvas

cLeft = (0)*cExprGain + cExprBias;
cRight = (S)*cExprGain + cExprBias;
cRange = ceil(cLeft):floor(cRight);
for s=1:S, % for each slice
  % Determine range of rows and columns applicable to current slice
  rTop = (s-1+0)*cExprGain + cExprBias;
  rBot = (s-1+1)*cExprGain + cExprBias;
  rRange = ceil(rTop):floor(rBot);
  
  % Generate linear combination of sines
  tRange = (cRange*tExprGain + tExprBias)/S;
  if any(tRange < 0) || any(tRange > 1),
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
