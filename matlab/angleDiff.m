function [C, C2]= angleDiff(A,B)
% A-B
    %diff = A-B;
    %C = diff;
    %C( abs(diff) >180 & diff >= 0 ) = diff( abs(diff) >180 & diff>= 0 ) - 360;
    %C( abs(diff) >180 & diff < 0 ) = diff( abs(diff) >180 & diff< 0 ) + 360;
    C = mod ( (( A - B ) + 180), 360 ) - 180; 
end