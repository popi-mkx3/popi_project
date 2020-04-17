function [H Ic_base F] = updateJSIM(inertia_props, force_transforms)

% Initialization of the composite-inertia matrices
Ic_EpauleAVD = inertia_props.lf_EpauleAVD.tensor6D;
Ic_HJambeAVD = inertia_props.lf_HJambeAVD.tensor6D;
Ic_BJambeAVD = inertia_props.lf_BJambeAVD.tensor6D;
Ic_EpauleAVG = inertia_props.lf_EpauleAVG.tensor6D;
Ic_HJambeAVG = inertia_props.lf_HJambeAVG.tensor6D;
Ic_BJambeAVG = inertia_props.lf_BJambeAVG.tensor6D;
Ic_EpauleARD = inertia_props.lf_EpauleARD.tensor6D;
Ic_HJambeARD = inertia_props.lf_HJambeARD.tensor6D;
Ic_BJambeARD = inertia_props.lf_BJambeARD.tensor6D;
Ic_EpauleARG = inertia_props.lf_EpauleARG.tensor6D;
Ic_HJambeARG = inertia_props.lf_HJambeARG.tensor6D;
Ic_BJambeARG = inertia_props.lf_BJambeARG.tensor6D;
Ic_base = inertia_props.lf_base.tensor6D;

% "Bottom-up" loop to update the inertia-composite property of each link,
%  for the current configuration

% Link BJambeARG
Ic_HJambeARG = Ic_HJambeARG + force_transforms.fr_HJambeARG_XF_fr_BJambeARG * Ic_BJambeARG * force_transforms.fr_HJambeARG_XF_fr_BJambeARG';

F(:,12) = Ic_BJambeARG(:,3);
H(12, 12) = F(3,12);

F(:,12) = force_transforms.fr_HJambeARG_XF_fr_BJambeARG * F(:,12);
H(11, 12) = H(12, 11) = F(3,12);

F(:,12) = force_transforms.fr_EpauleARG_XF_fr_HJambeARG * F(:,12);
H(10, 12) = H(12, 10) = F(3,12);

F(:,12) = force_transforms.fr_base_XF_fr_EpauleARG * F(:,12);

% Link HJambeARG
Ic_EpauleARG = Ic_EpauleARG + force_transforms.fr_EpauleARG_XF_fr_HJambeARG * Ic_HJambeARG * force_transforms.fr_EpauleARG_XF_fr_HJambeARG';

F(:,11) = Ic_HJambeARG(:,3);
H(11, 11) = F(3,11);

F(:,11) = force_transforms.fr_EpauleARG_XF_fr_HJambeARG * F(:,11);
H(10, 11) = H(11, 10) = F(3,11);

F(:,11) = force_transforms.fr_base_XF_fr_EpauleARG * F(:,11);

% Link EpauleARG
Ic_base = Ic_base + force_transforms.fr_base_XF_fr_EpauleARG * Ic_EpauleARG * force_transforms.fr_base_XF_fr_EpauleARG';

F(:,10) = Ic_EpauleARG(:,3);
H(10, 10) = F(3,10);

F(:,10) = force_transforms.fr_base_XF_fr_EpauleARG * F(:,10);

% Link BJambeARD
Ic_HJambeARD = Ic_HJambeARD + force_transforms.fr_HJambeARD_XF_fr_BJambeARD * Ic_BJambeARD * force_transforms.fr_HJambeARD_XF_fr_BJambeARD';

F(:,9) = Ic_BJambeARD(:,3);
H(9, 9) = F(3,9);

F(:,9) = force_transforms.fr_HJambeARD_XF_fr_BJambeARD * F(:,9);
H(8, 9) = H(9, 8) = F(3,9);

F(:,9) = force_transforms.fr_EpauleARD_XF_fr_HJambeARD * F(:,9);
H(7, 9) = H(9, 7) = F(3,9);

F(:,9) = force_transforms.fr_base_XF_fr_EpauleARD * F(:,9);

% Link HJambeARD
Ic_EpauleARD = Ic_EpauleARD + force_transforms.fr_EpauleARD_XF_fr_HJambeARD * Ic_HJambeARD * force_transforms.fr_EpauleARD_XF_fr_HJambeARD';

F(:,8) = Ic_HJambeARD(:,3);
H(8, 8) = F(3,8);

F(:,8) = force_transforms.fr_EpauleARD_XF_fr_HJambeARD * F(:,8);
H(7, 8) = H(8, 7) = F(3,8);

F(:,8) = force_transforms.fr_base_XF_fr_EpauleARD * F(:,8);

% Link EpauleARD
Ic_base = Ic_base + force_transforms.fr_base_XF_fr_EpauleARD * Ic_EpauleARD * force_transforms.fr_base_XF_fr_EpauleARD';

F(:,7) = Ic_EpauleARD(:,3);
H(7, 7) = F(3,7);

F(:,7) = force_transforms.fr_base_XF_fr_EpauleARD * F(:,7);

% Link BJambeAVG
Ic_HJambeAVG = Ic_HJambeAVG + force_transforms.fr_HJambeAVG_XF_fr_BJambeAVG * Ic_BJambeAVG * force_transforms.fr_HJambeAVG_XF_fr_BJambeAVG';

F(:,6) = Ic_BJambeAVG(:,3);
H(6, 6) = F(3,6);

F(:,6) = force_transforms.fr_HJambeAVG_XF_fr_BJambeAVG * F(:,6);
H(5, 6) = H(6, 5) = F(3,6);

F(:,6) = force_transforms.fr_EpauleAVG_XF_fr_HJambeAVG * F(:,6);
H(4, 6) = H(6, 4) = F(3,6);

F(:,6) = force_transforms.fr_base_XF_fr_EpauleAVG * F(:,6);

% Link HJambeAVG
Ic_EpauleAVG = Ic_EpauleAVG + force_transforms.fr_EpauleAVG_XF_fr_HJambeAVG * Ic_HJambeAVG * force_transforms.fr_EpauleAVG_XF_fr_HJambeAVG';

F(:,5) = Ic_HJambeAVG(:,3);
H(5, 5) = F(3,5);

F(:,5) = force_transforms.fr_EpauleAVG_XF_fr_HJambeAVG * F(:,5);
H(4, 5) = H(5, 4) = F(3,5);

F(:,5) = force_transforms.fr_base_XF_fr_EpauleAVG * F(:,5);

% Link EpauleAVG
Ic_base = Ic_base + force_transforms.fr_base_XF_fr_EpauleAVG * Ic_EpauleAVG * force_transforms.fr_base_XF_fr_EpauleAVG';

F(:,4) = Ic_EpauleAVG(:,3);
H(4, 4) = F(3,4);

F(:,4) = force_transforms.fr_base_XF_fr_EpauleAVG * F(:,4);

% Link BJambeAVD
Ic_HJambeAVD = Ic_HJambeAVD + force_transforms.fr_HJambeAVD_XF_fr_BJambeAVD * Ic_BJambeAVD * force_transforms.fr_HJambeAVD_XF_fr_BJambeAVD';

F(:,3) = Ic_BJambeAVD(:,3);
H(3, 3) = F(3,3);

F(:,3) = force_transforms.fr_HJambeAVD_XF_fr_BJambeAVD * F(:,3);
H(2, 3) = H(3, 2) = F(3,3);

F(:,3) = force_transforms.fr_EpauleAVD_XF_fr_HJambeAVD * F(:,3);
H(1, 3) = H(3, 1) = F(3,3);

F(:,3) = force_transforms.fr_base_XF_fr_EpauleAVD * F(:,3);

% Link HJambeAVD
Ic_EpauleAVD = Ic_EpauleAVD + force_transforms.fr_EpauleAVD_XF_fr_HJambeAVD * Ic_HJambeAVD * force_transforms.fr_EpauleAVD_XF_fr_HJambeAVD';

F(:,2) = Ic_HJambeAVD(:,3);
H(2, 2) = F(3,2);

F(:,2) = force_transforms.fr_EpauleAVD_XF_fr_HJambeAVD * F(:,2);
H(1, 2) = H(2, 1) = F(3,2);

F(:,2) = force_transforms.fr_base_XF_fr_EpauleAVD * F(:,2);

% Link EpauleAVD
Ic_base = Ic_base + force_transforms.fr_base_XF_fr_EpauleAVD * Ic_EpauleAVD * force_transforms.fr_base_XF_fr_EpauleAVD';

F(:,1) = Ic_EpauleAVD(:,3);
H(1, 1) = F(3,1);

F(:,1) = force_transforms.fr_base_XF_fr_EpauleAVD * F(:,1);
