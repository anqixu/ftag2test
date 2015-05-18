function [b] = greycode(a, inverse)
%

if numel(a) > 1,
  b = [];

  if inverse,
    mask = [0, a(1:end-1)];
    b = a;
    while any(mask ~= 0),
      b = xor(b, mask);
      mask = [0, mask(1:end-1)];
    end;
  else
    c = [0, a(1:end-1)];
    b = xor(c, a);
  end;
else
  b = 0;

  if inverse,
    mask = bitshift(a, -1);
    b = a;
    while mask ~= 0,
      b = bitxor(b, mask);
      mask = bitshift(mask, -1);
    end;
  else
    c = bitshift(a, -1);
    b = bitxor(c, a);
  end;
end;

end
