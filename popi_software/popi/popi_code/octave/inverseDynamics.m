function [tau base_a] = inverseDynamics(ip, xm, base_v, gravity, qd, qdd, fext)
if nargin < 7
    fext{1} = zeros(6,1);
    fext{2} = zeros(6,1);
    fext{3} = zeros(6,1);
    fext{4} = zeros(6,1);
    fext{5} = zeros(6,1);
    fext{6} = zeros(6,1);
    fext{7} = zeros(6,1);
    fext{8} = zeros(6,1);
    fext{9} = zeros(6,1);
    fext{10} = zeros(6,1);
    fext{11} = zeros(6,1);
    fext{12} = zeros(6,1);
    fext{13} = zeros(6,1);
end
%
% Pass 1. Forward propagate velocities and accelerations
%

% Link 'EpauleAVD'
EpauleAVD_v = xm.fr_EpauleAVD_XM_fr_base * base_v;
EpauleAVD_v(3) = EpauleAVD_v(3) + qd(1);

vcross = vcross_mx(EpauleAVD_v);

EpauleAVD_a = vcross(:,3) * qd(1);
EpauleAVD_a(3) = EpauleAVD_a(3) + qdd(1);

EpauleAVD_f = -fext{2} + ip.lf_EpauleAVD.tensor6D * EpauleAVD_a + (-vcross' * ip.lf_EpauleAVD.tensor6D * EpauleAVD_v);

% Link 'HJambeAVD'
HJambeAVD_v = xm.fr_HJambeAVD_XM_fr_EpauleAVD * EpauleAVD_v;
HJambeAVD_v(3) = HJambeAVD_v(3) + qd(2);

vcross = vcross_mx(HJambeAVD_v);

HJambeAVD_a = xm.fr_HJambeAVD_XM_fr_EpauleAVD * EpauleAVD_a + (vcross(:,3) * qd(2));
HJambeAVD_a(3) = HJambeAVD_a(3) + qdd(2);

HJambeAVD_f = -fext{3} + ip.lf_HJambeAVD.tensor6D * HJambeAVD_a + (-vcross' * ip.lf_HJambeAVD.tensor6D * HJambeAVD_v);

% Link 'BJambeAVD'
BJambeAVD_v = xm.fr_BJambeAVD_XM_fr_HJambeAVD * HJambeAVD_v;
BJambeAVD_v(3) = BJambeAVD_v(3) + qd(3);

vcross = vcross_mx(BJambeAVD_v);

BJambeAVD_a = xm.fr_BJambeAVD_XM_fr_HJambeAVD * HJambeAVD_a + (vcross(:,3) * qd(3));
BJambeAVD_a(3) = BJambeAVD_a(3) + qdd(3);

BJambeAVD_f = -fext{4} + ip.lf_BJambeAVD.tensor6D * BJambeAVD_a + (-vcross' * ip.lf_BJambeAVD.tensor6D * BJambeAVD_v);

% Link 'EpauleAVG'
EpauleAVG_v = xm.fr_EpauleAVG_XM_fr_base * base_v;
EpauleAVG_v(3) = EpauleAVG_v(3) + qd(4);

vcross = vcross_mx(EpauleAVG_v);

EpauleAVG_a = vcross(:,3) * qd(4);
EpauleAVG_a(3) = EpauleAVG_a(3) + qdd(4);

EpauleAVG_f = -fext{5} + ip.lf_EpauleAVG.tensor6D * EpauleAVG_a + (-vcross' * ip.lf_EpauleAVG.tensor6D * EpauleAVG_v);

% Link 'HJambeAVG'
HJambeAVG_v = xm.fr_HJambeAVG_XM_fr_EpauleAVG * EpauleAVG_v;
HJambeAVG_v(3) = HJambeAVG_v(3) + qd(5);

vcross = vcross_mx(HJambeAVG_v);

HJambeAVG_a = xm.fr_HJambeAVG_XM_fr_EpauleAVG * EpauleAVG_a + (vcross(:,3) * qd(5));
HJambeAVG_a(3) = HJambeAVG_a(3) + qdd(5);

HJambeAVG_f = -fext{6} + ip.lf_HJambeAVG.tensor6D * HJambeAVG_a + (-vcross' * ip.lf_HJambeAVG.tensor6D * HJambeAVG_v);

% Link 'BJambeAVG'
BJambeAVG_v = xm.fr_BJambeAVG_XM_fr_HJambeAVG * HJambeAVG_v;
BJambeAVG_v(3) = BJambeAVG_v(3) + qd(6);

vcross = vcross_mx(BJambeAVG_v);

BJambeAVG_a = xm.fr_BJambeAVG_XM_fr_HJambeAVG * HJambeAVG_a + (vcross(:,3) * qd(6));
BJambeAVG_a(3) = BJambeAVG_a(3) + qdd(6);

BJambeAVG_f = -fext{7} + ip.lf_BJambeAVG.tensor6D * BJambeAVG_a + (-vcross' * ip.lf_BJambeAVG.tensor6D * BJambeAVG_v);

% Link 'EpauleARD'
EpauleARD_v = xm.fr_EpauleARD_XM_fr_base * base_v;
EpauleARD_v(3) = EpauleARD_v(3) + qd(7);

vcross = vcross_mx(EpauleARD_v);

EpauleARD_a = vcross(:,3) * qd(7);
EpauleARD_a(3) = EpauleARD_a(3) + qdd(7);

EpauleARD_f = -fext{8} + ip.lf_EpauleARD.tensor6D * EpauleARD_a + (-vcross' * ip.lf_EpauleARD.tensor6D * EpauleARD_v);

% Link 'HJambeARD'
HJambeARD_v = xm.fr_HJambeARD_XM_fr_EpauleARD * EpauleARD_v;
HJambeARD_v(3) = HJambeARD_v(3) + qd(8);

vcross = vcross_mx(HJambeARD_v);

HJambeARD_a = xm.fr_HJambeARD_XM_fr_EpauleARD * EpauleARD_a + (vcross(:,3) * qd(8));
HJambeARD_a(3) = HJambeARD_a(3) + qdd(8);

HJambeARD_f = -fext{9} + ip.lf_HJambeARD.tensor6D * HJambeARD_a + (-vcross' * ip.lf_HJambeARD.tensor6D * HJambeARD_v);

% Link 'BJambeARD'
BJambeARD_v = xm.fr_BJambeARD_XM_fr_HJambeARD * HJambeARD_v;
BJambeARD_v(3) = BJambeARD_v(3) + qd(9);

vcross = vcross_mx(BJambeARD_v);

BJambeARD_a = xm.fr_BJambeARD_XM_fr_HJambeARD * HJambeARD_a + (vcross(:,3) * qd(9));
BJambeARD_a(3) = BJambeARD_a(3) + qdd(9);

BJambeARD_f = -fext{10} + ip.lf_BJambeARD.tensor6D * BJambeARD_a + (-vcross' * ip.lf_BJambeARD.tensor6D * BJambeARD_v);

% Link 'EpauleARG'
EpauleARG_v = xm.fr_EpauleARG_XM_fr_base * base_v;
EpauleARG_v(3) = EpauleARG_v(3) + qd(10);

vcross = vcross_mx(EpauleARG_v);

EpauleARG_a = vcross(:,3) * qd(10);
EpauleARG_a(3) = EpauleARG_a(3) + qdd(10);

EpauleARG_f = -fext{11} + ip.lf_EpauleARG.tensor6D * EpauleARG_a + (-vcross' * ip.lf_EpauleARG.tensor6D * EpauleARG_v);

% Link 'HJambeARG'
HJambeARG_v = xm.fr_HJambeARG_XM_fr_EpauleARG * EpauleARG_v;
HJambeARG_v(3) = HJambeARG_v(3) + qd(11);

vcross = vcross_mx(HJambeARG_v);

HJambeARG_a = xm.fr_HJambeARG_XM_fr_EpauleARG * EpauleARG_a + (vcross(:,3) * qd(11));
HJambeARG_a(3) = HJambeARG_a(3) + qdd(11);

HJambeARG_f = -fext{12} + ip.lf_HJambeARG.tensor6D * HJambeARG_a + (-vcross' * ip.lf_HJambeARG.tensor6D * HJambeARG_v);

% Link 'BJambeARG'
BJambeARG_v = xm.fr_BJambeARG_XM_fr_HJambeARG * HJambeARG_v;
BJambeARG_v(3) = BJambeARG_v(3) + qd(12);

vcross = vcross_mx(BJambeARG_v);

BJambeARG_a = xm.fr_BJambeARG_XM_fr_HJambeARG * HJambeARG_a + (vcross(:,3) * qd(12));
BJambeARG_a(3) = BJambeARG_a(3) + qdd(12);

BJambeARG_f = -fext{13} + ip.lf_BJambeARG.tensor6D * BJambeARG_a + (-vcross' * ip.lf_BJambeARG.tensor6D * BJambeARG_v);

%
% The force exerted on the floating base by the links
%
vcross = vcross_mx(base_v);
base_f = -fext{1} - vcross' * ip.lf_base.tensor6D * base_v;


%
% Pass 2. Compute the composite inertia and the spatial forces
%
ci = compositeInertia(ip, xm, 'motion');
HJambeARG_f = HJambeARG_f + xm.fr_BJambeARG_XM_fr_HJambeARG' * BJambeARG_f;
EpauleARG_f = EpauleARG_f + xm.fr_HJambeARG_XM_fr_EpauleARG' * HJambeARG_f;
base_f = base_f + xm.fr_EpauleARG_XM_fr_base' * EpauleARG_f;
HJambeARD_f = HJambeARD_f + xm.fr_BJambeARD_XM_fr_HJambeARD' * BJambeARD_f;
EpauleARD_f = EpauleARD_f + xm.fr_HJambeARD_XM_fr_EpauleARD' * HJambeARD_f;
base_f = base_f + xm.fr_EpauleARD_XM_fr_base' * EpauleARD_f;
HJambeAVG_f = HJambeAVG_f + xm.fr_BJambeAVG_XM_fr_HJambeAVG' * BJambeAVG_f;
EpauleAVG_f = EpauleAVG_f + xm.fr_HJambeAVG_XM_fr_EpauleAVG' * HJambeAVG_f;
base_f = base_f + xm.fr_EpauleAVG_XM_fr_base' * EpauleAVG_f;
HJambeAVD_f = HJambeAVD_f + xm.fr_BJambeAVD_XM_fr_HJambeAVD' * BJambeAVD_f;
EpauleAVD_f = EpauleAVD_f + xm.fr_HJambeAVD_XM_fr_EpauleAVD' * HJambeAVD_f;
base_f = base_f + xm.fr_EpauleAVD_XM_fr_base' * EpauleAVD_f;

%
% The base acceleration due to the force due to the movement of the links
%
base_a = - inverse(ci.base_Ic) * base_f; % TODO inverse

%
% Pass 3. Compute the joint forces while propagating back the floating base acceleration
%
tau = zeros(12, 1);
EpauleAVD_a = xm.fr_EpauleAVD_XM_fr_base * base_a;
tau(1) = ci.EpauleAVD_Ic(3,:) * EpauleAVD_a + EpauleAVD_f(3);

HJambeAVD_a = xm.fr_HJambeAVD_XM_fr_EpauleAVD * EpauleAVD_a;
tau(2) = ci.HJambeAVD_Ic(3,:) * HJambeAVD_a + HJambeAVD_f(3);

BJambeAVD_a = xm.fr_BJambeAVD_XM_fr_HJambeAVD * HJambeAVD_a;
tau(3) = ci.BJambeAVD_Ic(3,:) * BJambeAVD_a + BJambeAVD_f(3);

EpauleAVG_a = xm.fr_EpauleAVG_XM_fr_base * base_a;
tau(4) = ci.EpauleAVG_Ic(3,:) * EpauleAVG_a + EpauleAVG_f(3);

HJambeAVG_a = xm.fr_HJambeAVG_XM_fr_EpauleAVG * EpauleAVG_a;
tau(5) = ci.HJambeAVG_Ic(3,:) * HJambeAVG_a + HJambeAVG_f(3);

BJambeAVG_a = xm.fr_BJambeAVG_XM_fr_HJambeAVG * HJambeAVG_a;
tau(6) = ci.BJambeAVG_Ic(3,:) * BJambeAVG_a + BJambeAVG_f(3);

EpauleARD_a = xm.fr_EpauleARD_XM_fr_base * base_a;
tau(7) = ci.EpauleARD_Ic(3,:) * EpauleARD_a + EpauleARD_f(3);

HJambeARD_a = xm.fr_HJambeARD_XM_fr_EpauleARD * EpauleARD_a;
tau(8) = ci.HJambeARD_Ic(3,:) * HJambeARD_a + HJambeARD_f(3);

BJambeARD_a = xm.fr_BJambeARD_XM_fr_HJambeARD * HJambeARD_a;
tau(9) = ci.BJambeARD_Ic(3,:) * BJambeARD_a + BJambeARD_f(3);

EpauleARG_a = xm.fr_EpauleARG_XM_fr_base * base_a;
tau(10) = ci.EpauleARG_Ic(3,:) * EpauleARG_a + EpauleARG_f(3);

HJambeARG_a = xm.fr_HJambeARG_XM_fr_EpauleARG * EpauleARG_a;
tau(11) = ci.HJambeARG_Ic(3,:) * HJambeARG_a + HJambeARG_f(3);

BJambeARG_a = xm.fr_BJambeARG_XM_fr_HJambeARG * HJambeARG_a;
tau(12) = ci.BJambeARG_Ic(3,:) * BJambeARG_a + BJambeARG_f(3);


base_a = base_a + gravity;
end

function vc = vcross_mx(v)
    vc = [   0    -v(3)  v(2)   0     0     0    ;
             v(3)  0    -v(1)   0     0     0    ;
            -v(2)  v(1)  0      0     0     0    ;
             0    -v(6)  v(5)   0    -v(3)  v(2) ;
             v(6)  0    -v(4)   v(3)  0    -v(1) ;
            -v(5)  v(4)  0     -v(2)  v(1)  0    ];
end
