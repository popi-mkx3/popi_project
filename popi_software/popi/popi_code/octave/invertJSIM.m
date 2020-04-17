function [Hinv L Linv] = invertJSIM(H)

% The following code computes the lower triangular matrix L such that
%  H = L' L  (LTL factorization)
% Then it computes the inverse of L and the inverse of H

% LTL factorization
L = tril(H); % lower triangular

% Joint lh_kfe_joint, index 12 :
L(12, 12) = sqrt(L(12, 12));
L(12, 11) = L(12, 11) / L(12, 12);
L(12, 10) = L(12, 10) / L(12, 12);
L(11, 11) = L(11, 11) - L(12, 11) * L(12, 11);
L(11, 10) = L(11, 10) - L(12, 11) * L(12, 10);
L(10, 10) = L(10, 10) - L(12, 10) * L(12, 10);

% Joint lh_hfe_joint, index 11 :
L(11, 11) = sqrt(L(11, 11));
L(11, 10) = L(11, 10) / L(11, 11);
L(10, 10) = L(10, 10) - L(11, 10) * L(11, 10);

% Joint lh_haa_joint, index 10 :
L(10, 10) = sqrt(L(10, 10));

% Joint rh_kfe_joint, index 9 :
L(9, 9) = sqrt(L(9, 9));
L(9, 8) = L(9, 8) / L(9, 9);
L(9, 7) = L(9, 7) / L(9, 9);
L(8, 8) = L(8, 8) - L(9, 8) * L(9, 8);
L(8, 7) = L(8, 7) - L(9, 8) * L(9, 7);
L(7, 7) = L(7, 7) - L(9, 7) * L(9, 7);

% Joint rh_hfe_joint, index 8 :
L(8, 8) = sqrt(L(8, 8));
L(8, 7) = L(8, 7) / L(8, 8);
L(7, 7) = L(7, 7) - L(8, 7) * L(8, 7);

% Joint rh_haa_joint, index 7 :
L(7, 7) = sqrt(L(7, 7));

% Joint lf_kfe_joint, index 6 :
L(6, 6) = sqrt(L(6, 6));
L(6, 5) = L(6, 5) / L(6, 6);
L(6, 4) = L(6, 4) / L(6, 6);
L(5, 5) = L(5, 5) - L(6, 5) * L(6, 5);
L(5, 4) = L(5, 4) - L(6, 5) * L(6, 4);
L(4, 4) = L(4, 4) - L(6, 4) * L(6, 4);

% Joint lf_hfe_joint, index 5 :
L(5, 5) = sqrt(L(5, 5));
L(5, 4) = L(5, 4) / L(5, 5);
L(4, 4) = L(4, 4) - L(5, 4) * L(5, 4);

% Joint lf_haa_joint, index 4 :
L(4, 4) = sqrt(L(4, 4));

% Joint rf_kfe_joint, index 3 :
L(3, 3) = sqrt(L(3, 3));
L(3, 2) = L(3, 2) / L(3, 3);
L(3, 1) = L(3, 1) / L(3, 3);
L(2, 2) = L(2, 2) - L(3, 2) * L(3, 2);
L(2, 1) = L(2, 1) - L(3, 2) * L(3, 1);
L(1, 1) = L(1, 1) - L(3, 1) * L(3, 1);

% Joint rf_hfe_joint, index 2 :
L(2, 2) = sqrt(L(2, 2));
L(2, 1) = L(2, 1) / L(2, 2);
L(1, 1) = L(1, 1) - L(2, 1) * L(2, 1);

% Joint rf_haa_joint, index 1 :
L(1, 1) = sqrt(L(1, 1));


% Inverse of L
Linv(1, 1) = 1 / L(1, 1);
Linv(2, 2) = 1 / L(2, 2);
Linv(3, 3) = 1 / L(3, 3);
Linv(4, 4) = 1 / L(4, 4);
Linv(5, 5) = 1 / L(5, 5);
Linv(6, 6) = 1 / L(6, 6);
Linv(7, 7) = 1 / L(7, 7);
Linv(8, 8) = 1 / L(8, 8);
Linv(9, 9) = 1 / L(9, 9);
Linv(10, 10) = 1 / L(10, 10);
Linv(11, 11) = 1 / L(11, 11);
Linv(12, 12) = 1 / L(12, 12);
Linv(2, 1) = - Linv(1, 1) * ((Linv(2, 2) * L(2, 1)) + 0);
Linv(3, 2) = - Linv(2, 2) * ((Linv(3, 3) * L(3, 2)) + 0);
Linv(3, 1) = - Linv(1, 1) * ((Linv(3, 2) * L(2, 1)) + (Linv(3, 3) * L(3, 1)) + 0);
Linv(5, 4) = - Linv(4, 4) * ((Linv(5, 5) * L(5, 4)) + 0);
Linv(6, 5) = - Linv(5, 5) * ((Linv(6, 6) * L(6, 5)) + 0);
Linv(6, 4) = - Linv(4, 4) * ((Linv(6, 5) * L(5, 4)) + (Linv(6, 6) * L(6, 4)) + 0);
Linv(8, 7) = - Linv(7, 7) * ((Linv(8, 8) * L(8, 7)) + 0);
Linv(9, 8) = - Linv(8, 8) * ((Linv(9, 9) * L(9, 8)) + 0);
Linv(9, 7) = - Linv(7, 7) * ((Linv(9, 8) * L(8, 7)) + (Linv(9, 9) * L(9, 7)) + 0);
Linv(11, 10) = - Linv(10, 10) * ((Linv(11, 11) * L(11, 10)) + 0);
Linv(12, 11) = - Linv(11, 11) * ((Linv(12, 12) * L(12, 11)) + 0);
Linv(12, 10) = - Linv(10, 10) * ((Linv(12, 11) * L(11, 10)) + (Linv(12, 12) * L(12, 10)) + 0);

% Inverse of H
Hinv(1, 1) =  + (Linv(1, 1) * Linv(1, 1));
Hinv(2, 2) =  + (Linv(2, 1) * Linv(2, 1)) + (Linv(2, 2) * Linv(2, 2));
Hinv(2, 1) =  + (Linv(2, 1) * Linv(1, 1));
Hinv(1, 2) = Hinv(2, 1);
Hinv(3, 3) =  + (Linv(3, 1) * Linv(3, 1)) + (Linv(3, 2) * Linv(3, 2)) + (Linv(3, 3) * Linv(3, 3));
Hinv(3, 2) =  + (Linv(3, 1) * Linv(2, 1)) + (Linv(3, 2) * Linv(2, 2));
Hinv(2, 3) = Hinv(3, 2);
Hinv(3, 1) =  + (Linv(3, 1) * Linv(1, 1));
Hinv(1, 3) = Hinv(3, 1);
Hinv(4, 4) =  + (Linv(4, 4) * Linv(4, 4));
Hinv(5, 5) =  + (Linv(5, 4) * Linv(5, 4)) + (Linv(5, 5) * Linv(5, 5));
Hinv(5, 4) =  + (Linv(5, 4) * Linv(4, 4));
Hinv(4, 5) = Hinv(5, 4);
Hinv(6, 6) =  + (Linv(6, 4) * Linv(6, 4)) + (Linv(6, 5) * Linv(6, 5)) + (Linv(6, 6) * Linv(6, 6));
Hinv(6, 5) =  + (Linv(6, 4) * Linv(5, 4)) + (Linv(6, 5) * Linv(5, 5));
Hinv(5, 6) = Hinv(6, 5);
Hinv(6, 4) =  + (Linv(6, 4) * Linv(4, 4));
Hinv(4, 6) = Hinv(6, 4);
Hinv(7, 7) =  + (Linv(7, 7) * Linv(7, 7));
Hinv(8, 8) =  + (Linv(8, 7) * Linv(8, 7)) + (Linv(8, 8) * Linv(8, 8));
Hinv(8, 7) =  + (Linv(8, 7) * Linv(7, 7));
Hinv(7, 8) = Hinv(8, 7);
Hinv(9, 9) =  + (Linv(9, 7) * Linv(9, 7)) + (Linv(9, 8) * Linv(9, 8)) + (Linv(9, 9) * Linv(9, 9));
Hinv(9, 8) =  + (Linv(9, 7) * Linv(8, 7)) + (Linv(9, 8) * Linv(8, 8));
Hinv(8, 9) = Hinv(9, 8);
Hinv(9, 7) =  + (Linv(9, 7) * Linv(7, 7));
Hinv(7, 9) = Hinv(9, 7);
Hinv(10, 10) =  + (Linv(10, 10) * Linv(10, 10));
Hinv(11, 11) =  + (Linv(11, 10) * Linv(11, 10)) + (Linv(11, 11) * Linv(11, 11));
Hinv(11, 10) =  + (Linv(11, 10) * Linv(10, 10));
Hinv(10, 11) = Hinv(11, 10);
Hinv(12, 12) =  + (Linv(12, 10) * Linv(12, 10)) + (Linv(12, 11) * Linv(12, 11)) + (Linv(12, 12) * Linv(12, 12));
Hinv(12, 11) =  + (Linv(12, 10) * Linv(11, 10)) + (Linv(12, 11) * Linv(11, 11));
Hinv(11, 12) = Hinv(12, 11);
Hinv(12, 10) =  + (Linv(12, 10) * Linv(10, 10));
Hinv(10, 12) = Hinv(12, 10);
