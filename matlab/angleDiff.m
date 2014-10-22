function C = angleDiff(A,B)
% A-B

    diff = A-B;
    C = diff;
    C( abs(diff) >180 & diff >= 0 ) = diff( abs(diff) >180 & diff>= 0 ) - 360;
    C( abs(diff) >180 & diff < 0 ) = diff( abs(diff) >180 & diff< 0 ) + 360;
end