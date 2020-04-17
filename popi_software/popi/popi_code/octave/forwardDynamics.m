function [qdd base_a] = forwardDynamics(ip, xm, base_v, gravity, qd, tau, fext)
qdd = zeros(12,1);
base_AI = ip.lf_base.tensor6D;
EpauleAVD_AI = ip.lf_EpauleAVD.tensor6D;
HJambeAVD_AI = ip.lf_HJambeAVD.tensor6D;
BJambeAVD_AI = ip.lf_BJambeAVD.tensor6D;
EpauleAVG_AI = ip.lf_EpauleAVG.tensor6D;
HJambeAVG_AI = ip.lf_HJambeAVG.tensor6D;
BJambeAVG_AI = ip.lf_BJambeAVG.tensor6D;
EpauleARD_AI = ip.lf_EpauleARD.tensor6D;
HJambeARD_AI = ip.lf_HJambeARD.tensor6D;
BJambeARD_AI = ip.lf_BJambeARD.tensor6D;
EpauleARG_AI = ip.lf_EpauleARG.tensor6D;
HJambeARG_AI = ip.lf_HJambeARG.tensor6D;
BJambeARG_AI = ip.lf_BJambeARG.tensor6D;
if nargin > 6
    base_p = - fext{1};
    EpauleAVD_p = - fext{2};
    HJambeAVD_p = - fext{3};
    BJambeAVD_p = - fext{4};
    EpauleAVG_p = - fext{5};
    HJambeAVG_p = - fext{6};
    BJambeAVG_p = - fext{7};
    EpauleARD_p = - fext{8};
    HJambeARD_p = - fext{9};
    BJambeARD_p = - fext{10};
    EpauleARG_p = - fext{11};
    HJambeARG_p = - fext{12};
    BJambeARG_p = - fext{13};
else
    base_p = zeros(6,1);
    EpauleAVD_p = zeros(6,1);
    HJambeAVD_p = zeros(6,1);
    BJambeAVD_p = zeros(6,1);
    EpauleAVG_p = zeros(6,1);
    HJambeAVG_p = zeros(6,1);
    BJambeAVG_p = zeros(6,1);
    EpauleARD_p = zeros(6,1);
    HJambeARD_p = zeros(6,1);
    BJambeARD_p = zeros(6,1);
    EpauleARG_p = zeros(6,1);
    HJambeARG_p = zeros(6,1);
    BJambeARG_p = zeros(6,1);
end
%% ---------------------- FIRST PASS ----------------------
%% Note that, during the first pass, the articulated inertias are really
%% just the spatial inertia of the links (see assignments above).
%%  Afterwards things change, and articulated inertias shall not be used
%%  in functions which work specifically with spatial inertias.

EpauleAVD_v = (xm.fr_EpauleAVD_XM_fr_base) * base_v;
EpauleAVD_v(3) = EpauleAVD_v(3) + qd(1);

vcross = vcross_mx(EpauleAVD_v);
EpauleAVD_c = vcross(:,3) * qd(1);
EpauleAVD_p = EpauleAVD_p + -vcross' * EpauleAVD_AI * EpauleAVD_v; %%% vxIv(EpauleAVD_v, EpauleAVD_AI);

HJambeAVD_v = (xm.fr_HJambeAVD_XM_fr_EpauleAVD) * EpauleAVD_v;
HJambeAVD_v(3) = HJambeAVD_v(3) + qd(2);

vcross = vcross_mx(HJambeAVD_v);
HJambeAVD_c = vcross(:,3) * qd(2);
HJambeAVD_p = HJambeAVD_p + -vcross' * HJambeAVD_AI * HJambeAVD_v; %%% vxIv(HJambeAVD_v, HJambeAVD_AI);

BJambeAVD_v = (xm.fr_BJambeAVD_XM_fr_HJambeAVD) * HJambeAVD_v;
BJambeAVD_v(3) = BJambeAVD_v(3) + qd(3);

vcross = vcross_mx(BJambeAVD_v);
BJambeAVD_c = vcross(:,3) * qd(3);
BJambeAVD_p = BJambeAVD_p + -vcross' * BJambeAVD_AI * BJambeAVD_v; %%% vxIv(BJambeAVD_v, BJambeAVD_AI);

EpauleAVG_v = (xm.fr_EpauleAVG_XM_fr_base) * base_v;
EpauleAVG_v(3) = EpauleAVG_v(3) + qd(4);

vcross = vcross_mx(EpauleAVG_v);
EpauleAVG_c = vcross(:,3) * qd(4);
EpauleAVG_p = EpauleAVG_p + -vcross' * EpauleAVG_AI * EpauleAVG_v; %%% vxIv(EpauleAVG_v, EpauleAVG_AI);

HJambeAVG_v = (xm.fr_HJambeAVG_XM_fr_EpauleAVG) * EpauleAVG_v;
HJambeAVG_v(3) = HJambeAVG_v(3) + qd(5);

vcross = vcross_mx(HJambeAVG_v);
HJambeAVG_c = vcross(:,3) * qd(5);
HJambeAVG_p = HJambeAVG_p + -vcross' * HJambeAVG_AI * HJambeAVG_v; %%% vxIv(HJambeAVG_v, HJambeAVG_AI);

BJambeAVG_v = (xm.fr_BJambeAVG_XM_fr_HJambeAVG) * HJambeAVG_v;
BJambeAVG_v(3) = BJambeAVG_v(3) + qd(6);

vcross = vcross_mx(BJambeAVG_v);
BJambeAVG_c = vcross(:,3) * qd(6);
BJambeAVG_p = BJambeAVG_p + -vcross' * BJambeAVG_AI * BJambeAVG_v; %%% vxIv(BJambeAVG_v, BJambeAVG_AI);

EpauleARD_v = (xm.fr_EpauleARD_XM_fr_base) * base_v;
EpauleARD_v(3) = EpauleARD_v(3) + qd(7);

vcross = vcross_mx(EpauleARD_v);
EpauleARD_c = vcross(:,3) * qd(7);
EpauleARD_p = EpauleARD_p + -vcross' * EpauleARD_AI * EpauleARD_v; %%% vxIv(EpauleARD_v, EpauleARD_AI);

HJambeARD_v = (xm.fr_HJambeARD_XM_fr_EpauleARD) * EpauleARD_v;
HJambeARD_v(3) = HJambeARD_v(3) + qd(8);

vcross = vcross_mx(HJambeARD_v);
HJambeARD_c = vcross(:,3) * qd(8);
HJambeARD_p = HJambeARD_p + -vcross' * HJambeARD_AI * HJambeARD_v; %%% vxIv(HJambeARD_v, HJambeARD_AI);

BJambeARD_v = (xm.fr_BJambeARD_XM_fr_HJambeARD) * HJambeARD_v;
BJambeARD_v(3) = BJambeARD_v(3) + qd(9);

vcross = vcross_mx(BJambeARD_v);
BJambeARD_c = vcross(:,3) * qd(9);
BJambeARD_p = BJambeARD_p + -vcross' * BJambeARD_AI * BJambeARD_v; %%% vxIv(BJambeARD_v, BJambeARD_AI);

EpauleARG_v = (xm.fr_EpauleARG_XM_fr_base) * base_v;
EpauleARG_v(3) = EpauleARG_v(3) + qd(10);

vcross = vcross_mx(EpauleARG_v);
EpauleARG_c = vcross(:,3) * qd(10);
EpauleARG_p = EpauleARG_p + -vcross' * EpauleARG_AI * EpauleARG_v; %%% vxIv(EpauleARG_v, EpauleARG_AI);

HJambeARG_v = (xm.fr_HJambeARG_XM_fr_EpauleARG) * EpauleARG_v;
HJambeARG_v(3) = HJambeARG_v(3) + qd(11);

vcross = vcross_mx(HJambeARG_v);
HJambeARG_c = vcross(:,3) * qd(11);
HJambeARG_p = HJambeARG_p + -vcross' * HJambeARG_AI * HJambeARG_v; %%% vxIv(HJambeARG_v, HJambeARG_AI);

BJambeARG_v = (xm.fr_BJambeARG_XM_fr_HJambeARG) * HJambeARG_v;
BJambeARG_v(3) = BJambeARG_v(3) + qd(12);

vcross = vcross_mx(BJambeARG_v);
BJambeARG_c = vcross(:,3) * qd(12);
BJambeARG_p = BJambeARG_p + -vcross' * BJambeARG_AI * BJambeARG_v; %%% vxIv(BJambeARG_v, BJambeARG_AI);

vcross = vcross_mx(base_v);
base_p = base_p + -vcross' * base_AI * base_v;

%% ---------------------- SECOND PASS ----------------------
IaB = zeros(6,6);
pa  = zeros(6,1);

BJambeARG_u = tau(12) - BJambeARG_p(3);
BJambeARG_U = BJambeARG_AI(:,3);
BJambeARG_D = BJambeARG_U(3);

Ia_r = BJambeARG_AI - BJambeARG_U/BJambeARG_D * BJambeARG_U';
pa = BJambeARG_p + Ia_r * BJambeARG_c + BJambeARG_U * BJambeARG_u/BJambeARG_D;
IaB = xm.fr_BJambeARG_XM_fr_HJambeARG' * Ia_r * xm.fr_BJambeARG_XM_fr_HJambeARG;          %%ctransform_Ia_revolute(Ia_r, xm.fr_BJambeARG_XM_fr_HJambeARG, IaB);
HJambeARG_AI = HJambeARG_AI + IaB;
HJambeARG_p = HJambeARG_p + (xm.fr_BJambeARG_XM_fr_HJambeARG)' * pa;

HJambeARG_u = tau(11) - HJambeARG_p(3);
HJambeARG_U = HJambeARG_AI(:,3);
HJambeARG_D = HJambeARG_U(3);

Ia_r = HJambeARG_AI - HJambeARG_U/HJambeARG_D * HJambeARG_U';
pa = HJambeARG_p + Ia_r * HJambeARG_c + HJambeARG_U * HJambeARG_u/HJambeARG_D;
IaB = xm.fr_HJambeARG_XM_fr_EpauleARG' * Ia_r * xm.fr_HJambeARG_XM_fr_EpauleARG;          %%ctransform_Ia_revolute(Ia_r, xm.fr_HJambeARG_XM_fr_EpauleARG, IaB);
EpauleARG_AI = EpauleARG_AI + IaB;
EpauleARG_p = EpauleARG_p + (xm.fr_HJambeARG_XM_fr_EpauleARG)' * pa;

EpauleARG_u = tau(10) - EpauleARG_p(3);
EpauleARG_U = EpauleARG_AI(:,3);
EpauleARG_D = EpauleARG_U(3);

Ia_r = EpauleARG_AI - EpauleARG_U/EpauleARG_D * EpauleARG_U';
pa = EpauleARG_p + Ia_r * EpauleARG_c + EpauleARG_U * EpauleARG_u/EpauleARG_D;
IaB = xm.fr_EpauleARG_XM_fr_base' * Ia_r * xm.fr_EpauleARG_XM_fr_base;          %%ctransform_Ia_revolute(Ia_r, xm.fr_EpauleARG_XM_fr_base, IaB);
base_AI = base_AI + IaB;
base_p = base_p + (xm.fr_EpauleARG_XM_fr_base)' * pa;

BJambeARD_u = tau(9) - BJambeARD_p(3);
BJambeARD_U = BJambeARD_AI(:,3);
BJambeARD_D = BJambeARD_U(3);

Ia_r = BJambeARD_AI - BJambeARD_U/BJambeARD_D * BJambeARD_U';
pa = BJambeARD_p + Ia_r * BJambeARD_c + BJambeARD_U * BJambeARD_u/BJambeARD_D;
IaB = xm.fr_BJambeARD_XM_fr_HJambeARD' * Ia_r * xm.fr_BJambeARD_XM_fr_HJambeARD;          %%ctransform_Ia_revolute(Ia_r, xm.fr_BJambeARD_XM_fr_HJambeARD, IaB);
HJambeARD_AI = HJambeARD_AI + IaB;
HJambeARD_p = HJambeARD_p + (xm.fr_BJambeARD_XM_fr_HJambeARD)' * pa;

HJambeARD_u = tau(8) - HJambeARD_p(3);
HJambeARD_U = HJambeARD_AI(:,3);
HJambeARD_D = HJambeARD_U(3);

Ia_r = HJambeARD_AI - HJambeARD_U/HJambeARD_D * HJambeARD_U';
pa = HJambeARD_p + Ia_r * HJambeARD_c + HJambeARD_U * HJambeARD_u/HJambeARD_D;
IaB = xm.fr_HJambeARD_XM_fr_EpauleARD' * Ia_r * xm.fr_HJambeARD_XM_fr_EpauleARD;          %%ctransform_Ia_revolute(Ia_r, xm.fr_HJambeARD_XM_fr_EpauleARD, IaB);
EpauleARD_AI = EpauleARD_AI + IaB;
EpauleARD_p = EpauleARD_p + (xm.fr_HJambeARD_XM_fr_EpauleARD)' * pa;

EpauleARD_u = tau(7) - EpauleARD_p(3);
EpauleARD_U = EpauleARD_AI(:,3);
EpauleARD_D = EpauleARD_U(3);

Ia_r = EpauleARD_AI - EpauleARD_U/EpauleARD_D * EpauleARD_U';
pa = EpauleARD_p + Ia_r * EpauleARD_c + EpauleARD_U * EpauleARD_u/EpauleARD_D;
IaB = xm.fr_EpauleARD_XM_fr_base' * Ia_r * xm.fr_EpauleARD_XM_fr_base;          %%ctransform_Ia_revolute(Ia_r, xm.fr_EpauleARD_XM_fr_base, IaB);
base_AI = base_AI + IaB;
base_p = base_p + (xm.fr_EpauleARD_XM_fr_base)' * pa;

BJambeAVG_u = tau(6) - BJambeAVG_p(3);
BJambeAVG_U = BJambeAVG_AI(:,3);
BJambeAVG_D = BJambeAVG_U(3);

Ia_r = BJambeAVG_AI - BJambeAVG_U/BJambeAVG_D * BJambeAVG_U';
pa = BJambeAVG_p + Ia_r * BJambeAVG_c + BJambeAVG_U * BJambeAVG_u/BJambeAVG_D;
IaB = xm.fr_BJambeAVG_XM_fr_HJambeAVG' * Ia_r * xm.fr_BJambeAVG_XM_fr_HJambeAVG;          %%ctransform_Ia_revolute(Ia_r, xm.fr_BJambeAVG_XM_fr_HJambeAVG, IaB);
HJambeAVG_AI = HJambeAVG_AI + IaB;
HJambeAVG_p = HJambeAVG_p + (xm.fr_BJambeAVG_XM_fr_HJambeAVG)' * pa;

HJambeAVG_u = tau(5) - HJambeAVG_p(3);
HJambeAVG_U = HJambeAVG_AI(:,3);
HJambeAVG_D = HJambeAVG_U(3);

Ia_r = HJambeAVG_AI - HJambeAVG_U/HJambeAVG_D * HJambeAVG_U';
pa = HJambeAVG_p + Ia_r * HJambeAVG_c + HJambeAVG_U * HJambeAVG_u/HJambeAVG_D;
IaB = xm.fr_HJambeAVG_XM_fr_EpauleAVG' * Ia_r * xm.fr_HJambeAVG_XM_fr_EpauleAVG;          %%ctransform_Ia_revolute(Ia_r, xm.fr_HJambeAVG_XM_fr_EpauleAVG, IaB);
EpauleAVG_AI = EpauleAVG_AI + IaB;
EpauleAVG_p = EpauleAVG_p + (xm.fr_HJambeAVG_XM_fr_EpauleAVG)' * pa;

EpauleAVG_u = tau(4) - EpauleAVG_p(3);
EpauleAVG_U = EpauleAVG_AI(:,3);
EpauleAVG_D = EpauleAVG_U(3);

Ia_r = EpauleAVG_AI - EpauleAVG_U/EpauleAVG_D * EpauleAVG_U';
pa = EpauleAVG_p + Ia_r * EpauleAVG_c + EpauleAVG_U * EpauleAVG_u/EpauleAVG_D;
IaB = xm.fr_EpauleAVG_XM_fr_base' * Ia_r * xm.fr_EpauleAVG_XM_fr_base;          %%ctransform_Ia_revolute(Ia_r, xm.fr_EpauleAVG_XM_fr_base, IaB);
base_AI = base_AI + IaB;
base_p = base_p + (xm.fr_EpauleAVG_XM_fr_base)' * pa;

BJambeAVD_u = tau(3) - BJambeAVD_p(3);
BJambeAVD_U = BJambeAVD_AI(:,3);
BJambeAVD_D = BJambeAVD_U(3);

Ia_r = BJambeAVD_AI - BJambeAVD_U/BJambeAVD_D * BJambeAVD_U';
pa = BJambeAVD_p + Ia_r * BJambeAVD_c + BJambeAVD_U * BJambeAVD_u/BJambeAVD_D;
IaB = xm.fr_BJambeAVD_XM_fr_HJambeAVD' * Ia_r * xm.fr_BJambeAVD_XM_fr_HJambeAVD;          %%ctransform_Ia_revolute(Ia_r, xm.fr_BJambeAVD_XM_fr_HJambeAVD, IaB);
HJambeAVD_AI = HJambeAVD_AI + IaB;
HJambeAVD_p = HJambeAVD_p + (xm.fr_BJambeAVD_XM_fr_HJambeAVD)' * pa;

HJambeAVD_u = tau(2) - HJambeAVD_p(3);
HJambeAVD_U = HJambeAVD_AI(:,3);
HJambeAVD_D = HJambeAVD_U(3);

Ia_r = HJambeAVD_AI - HJambeAVD_U/HJambeAVD_D * HJambeAVD_U';
pa = HJambeAVD_p + Ia_r * HJambeAVD_c + HJambeAVD_U * HJambeAVD_u/HJambeAVD_D;
IaB = xm.fr_HJambeAVD_XM_fr_EpauleAVD' * Ia_r * xm.fr_HJambeAVD_XM_fr_EpauleAVD;          %%ctransform_Ia_revolute(Ia_r, xm.fr_HJambeAVD_XM_fr_EpauleAVD, IaB);
EpauleAVD_AI = EpauleAVD_AI + IaB;
EpauleAVD_p = EpauleAVD_p + (xm.fr_HJambeAVD_XM_fr_EpauleAVD)' * pa;

EpauleAVD_u = tau(1) - EpauleAVD_p(3);
EpauleAVD_U = EpauleAVD_AI(:,3);
EpauleAVD_D = EpauleAVD_U(3);

Ia_r = EpauleAVD_AI - EpauleAVD_U/EpauleAVD_D * EpauleAVD_U';
pa = EpauleAVD_p + Ia_r * EpauleAVD_c + EpauleAVD_U * EpauleAVD_u/EpauleAVD_D;
IaB = xm.fr_EpauleAVD_XM_fr_base' * Ia_r * xm.fr_EpauleAVD_XM_fr_base;          %%ctransform_Ia_revolute(Ia_r, xm.fr_EpauleAVD_XM_fr_base, IaB);
base_AI = base_AI + IaB;
base_p = base_p + (xm.fr_EpauleAVD_XM_fr_base)' * pa;

% + The acceleration of the floating base base, without gravity
base_a = - base_AI \ base_p;

% ---------------------- THIRD PASS ----------------------
EpauleAVD_a = (xm.fr_EpauleAVD_XM_fr_base) * base_a + EpauleAVD_c;
qdd(1) = (EpauleAVD_u - dot(EpauleAVD_U,EpauleAVD_a)) / EpauleAVD_D;
EpauleAVD_a(3) = EpauleAVD_a(3) + qdd(1);
HJambeAVD_a = (xm.fr_HJambeAVD_XM_fr_EpauleAVD) * EpauleAVD_a + HJambeAVD_c;
qdd(2) = (HJambeAVD_u - dot(HJambeAVD_U,HJambeAVD_a)) / HJambeAVD_D;
HJambeAVD_a(3) = HJambeAVD_a(3) + qdd(2);
BJambeAVD_a = (xm.fr_BJambeAVD_XM_fr_HJambeAVD) * HJambeAVD_a + BJambeAVD_c;
qdd(3) = (BJambeAVD_u - dot(BJambeAVD_U,BJambeAVD_a)) / BJambeAVD_D;
BJambeAVD_a(3) = BJambeAVD_a(3) + qdd(3);
EpauleAVG_a = (xm.fr_EpauleAVG_XM_fr_base) * base_a + EpauleAVG_c;
qdd(4) = (EpauleAVG_u - dot(EpauleAVG_U,EpauleAVG_a)) / EpauleAVG_D;
EpauleAVG_a(3) = EpauleAVG_a(3) + qdd(4);
HJambeAVG_a = (xm.fr_HJambeAVG_XM_fr_EpauleAVG) * EpauleAVG_a + HJambeAVG_c;
qdd(5) = (HJambeAVG_u - dot(HJambeAVG_U,HJambeAVG_a)) / HJambeAVG_D;
HJambeAVG_a(3) = HJambeAVG_a(3) + qdd(5);
BJambeAVG_a = (xm.fr_BJambeAVG_XM_fr_HJambeAVG) * HJambeAVG_a + BJambeAVG_c;
qdd(6) = (BJambeAVG_u - dot(BJambeAVG_U,BJambeAVG_a)) / BJambeAVG_D;
BJambeAVG_a(3) = BJambeAVG_a(3) + qdd(6);
EpauleARD_a = (xm.fr_EpauleARD_XM_fr_base) * base_a + EpauleARD_c;
qdd(7) = (EpauleARD_u - dot(EpauleARD_U,EpauleARD_a)) / EpauleARD_D;
EpauleARD_a(3) = EpauleARD_a(3) + qdd(7);
HJambeARD_a = (xm.fr_HJambeARD_XM_fr_EpauleARD) * EpauleARD_a + HJambeARD_c;
qdd(8) = (HJambeARD_u - dot(HJambeARD_U,HJambeARD_a)) / HJambeARD_D;
HJambeARD_a(3) = HJambeARD_a(3) + qdd(8);
BJambeARD_a = (xm.fr_BJambeARD_XM_fr_HJambeARD) * HJambeARD_a + BJambeARD_c;
qdd(9) = (BJambeARD_u - dot(BJambeARD_U,BJambeARD_a)) / BJambeARD_D;
BJambeARD_a(3) = BJambeARD_a(3) + qdd(9);
EpauleARG_a = (xm.fr_EpauleARG_XM_fr_base) * base_a + EpauleARG_c;
qdd(10) = (EpauleARG_u - dot(EpauleARG_U,EpauleARG_a)) / EpauleARG_D;
EpauleARG_a(3) = EpauleARG_a(3) + qdd(10);
HJambeARG_a = (xm.fr_HJambeARG_XM_fr_EpauleARG) * EpauleARG_a + HJambeARG_c;
qdd(11) = (HJambeARG_u - dot(HJambeARG_U,HJambeARG_a)) / HJambeARG_D;
HJambeARG_a(3) = HJambeARG_a(3) + qdd(11);
BJambeARG_a = (xm.fr_BJambeARG_XM_fr_HJambeARG) * HJambeARG_a + BJambeARG_c;
qdd(12) = (BJambeARG_u - dot(BJambeARG_U,BJambeARG_a)) / BJambeARG_D;
BJambeARG_a(3) = BJambeARG_a(3) + qdd(12);

% + Add gravity to the acceleration of the floating base
base_a = base_a + gravity;
end

function ret = vxIv(omegaz, I)
    wz2 = omegaz*omegaz;
    ret = zeros(6,1);
    ret(1) = -I(2,3) * wz2;
    ret(2) =  I(1,2) * wz2;
    %%ret(3) =  0;
    ret(4) =  I(2,6) * wz2;
    ret(5) =  I(3,4) * wz2;
    %%ret(6) =  0;
end

function vc = vcross_mx(v)
    vc = [   0    -v(3)  v(2)   0     0     0    ;
             v(3)  0    -v(1)   0     0     0    ;
            -v(2)  v(1)  0      0     0     0    ;
             0    -v(6)  v(5)   0    -v(3)  v(2) ;
             v(6)  0    -v(4)   v(3)  0    -v(1) ;
            -v(5)  v(4)  0     -v(2)  v(1)  0    ];
end
