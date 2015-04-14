function tagPadded = padTagBorder(tag, tagPaddedWidthPx, padPxVal)
% Pad a constant-color border to a square, grayscale (uint8) tag image

if nargin < 3,
  padPxVal = uint8(255);
else
  padPxVal = uint8(padPxVal);
end

tagWidthPx = size(tag, 2);
if size(tag, 1) ~= tagWidthPx,
  error('padTagBorder:InvalidArg', 'Input tag image is not square');
elseif tagWidthPx >= tagPaddedWidthPx,
  erorr('padTagBorder:InvalidArg', 'Input tag image has larger width than specified padded width');
end

tagPadded = padPxVal*ones(tagPaddedWidthPx, tagPaddedWidthPx, 'uint8');
padSideWidthPx = ceil((tagPaddedWidthPx-tagWidthPx)/2)-1;
padRange = padSideWidthPx+(1:tagWidthPx);
tagPadded(padRange, padRange) = tag;

end
