function ci = compositeInertia(ip, xf, transformsType)

% Computes the spatial composite inertia of each link of the robot.
% Arguments:
% - ip : the structure with the inertia properties
% - xf : the structure with the spatial coordinate transformation matrices
% - transformsType : a string specifying which is the type of the given
%      coordinate transforms, either velocity ('motion') or force ('force').
%      Optional argument, default is 'force'.

if nargin < 3
    transformsType = 'force';
end

%
% Initialization of the composite-inertia matrices
%
ci.EpauleAVD_Ic = ip.lf_EpauleAVD.tensor6D;
ci.HJambeAVD_Ic = ip.lf_HJambeAVD.tensor6D;
ci.BJambeAVD_Ic = ip.lf_BJambeAVD.tensor6D;
ci.EpauleAVG_Ic = ip.lf_EpauleAVG.tensor6D;
ci.HJambeAVG_Ic = ip.lf_HJambeAVG.tensor6D;
ci.BJambeAVG_Ic = ip.lf_BJambeAVG.tensor6D;
ci.EpauleARD_Ic = ip.lf_EpauleARD.tensor6D;
ci.HJambeARD_Ic = ip.lf_HJambeARD.tensor6D;
ci.BJambeARD_Ic = ip.lf_BJambeARD.tensor6D;
ci.EpauleARG_Ic = ip.lf_EpauleARG.tensor6D;
ci.HJambeARG_Ic = ip.lf_HJambeARG.tensor6D;
ci.BJambeARG_Ic = ip.lf_BJambeARG.tensor6D;
ci.base_Ic = ip.lf_base.tensor6D;

%
% Leafs-to-root pass to update the composite inertia of
%     each link, for the current configuration:
%

if strcmp(transformsType, 'motion')  % we have transforms for motion vectors

% Contribution of link BJambeARG
ci.HJambeARG_Ic = ci.HJambeARG_Ic + xf.fr_BJambeARG_XM_fr_HJambeARG' * ci.BJambeARG_Ic * xf.fr_BJambeARG_XM_fr_HJambeARG;


% Contribution of link HJambeARG
ci.EpauleARG_Ic = ci.EpauleARG_Ic + xf.fr_HJambeARG_XM_fr_EpauleARG' * ci.HJambeARG_Ic * xf.fr_HJambeARG_XM_fr_EpauleARG;


% Contribution of link EpauleARG
ci.base_Ic = ci.base_Ic + xf.fr_EpauleARG_XM_fr_base' * ci.EpauleARG_Ic * xf.fr_EpauleARG_XM_fr_base;


% Contribution of link BJambeARD
ci.HJambeARD_Ic = ci.HJambeARD_Ic + xf.fr_BJambeARD_XM_fr_HJambeARD' * ci.BJambeARD_Ic * xf.fr_BJambeARD_XM_fr_HJambeARD;


% Contribution of link HJambeARD
ci.EpauleARD_Ic = ci.EpauleARD_Ic + xf.fr_HJambeARD_XM_fr_EpauleARD' * ci.HJambeARD_Ic * xf.fr_HJambeARD_XM_fr_EpauleARD;


% Contribution of link EpauleARD
ci.base_Ic = ci.base_Ic + xf.fr_EpauleARD_XM_fr_base' * ci.EpauleARD_Ic * xf.fr_EpauleARD_XM_fr_base;


% Contribution of link BJambeAVG
ci.HJambeAVG_Ic = ci.HJambeAVG_Ic + xf.fr_BJambeAVG_XM_fr_HJambeAVG' * ci.BJambeAVG_Ic * xf.fr_BJambeAVG_XM_fr_HJambeAVG;


% Contribution of link HJambeAVG
ci.EpauleAVG_Ic = ci.EpauleAVG_Ic + xf.fr_HJambeAVG_XM_fr_EpauleAVG' * ci.HJambeAVG_Ic * xf.fr_HJambeAVG_XM_fr_EpauleAVG;


% Contribution of link EpauleAVG
ci.base_Ic = ci.base_Ic + xf.fr_EpauleAVG_XM_fr_base' * ci.EpauleAVG_Ic * xf.fr_EpauleAVG_XM_fr_base;


% Contribution of link BJambeAVD
ci.HJambeAVD_Ic = ci.HJambeAVD_Ic + xf.fr_BJambeAVD_XM_fr_HJambeAVD' * ci.BJambeAVD_Ic * xf.fr_BJambeAVD_XM_fr_HJambeAVD;


% Contribution of link HJambeAVD
ci.EpauleAVD_Ic = ci.EpauleAVD_Ic + xf.fr_HJambeAVD_XM_fr_EpauleAVD' * ci.HJambeAVD_Ic * xf.fr_HJambeAVD_XM_fr_EpauleAVD;


% Contribution of link EpauleAVD
ci.base_Ic = ci.base_Ic + xf.fr_EpauleAVD_XM_fr_base' * ci.EpauleAVD_Ic * xf.fr_EpauleAVD_XM_fr_base;


else % we have transforms for force vectors

% Contribution of link BJambeARG
ci.HJambeARG_Ic = ci.HJambeARG_Ic + xf.fr_HJambeARG_XF_fr_BJambeARG * ci.BJambeARG_Ic * xf.fr_HJambeARG_XF_fr_BJambeARG';


% Contribution of link HJambeARG
ci.EpauleARG_Ic = ci.EpauleARG_Ic + xf.fr_EpauleARG_XF_fr_HJambeARG * ci.HJambeARG_Ic * xf.fr_EpauleARG_XF_fr_HJambeARG';


% Contribution of link EpauleARG
ci.base_Ic = ci.base_Ic + xf.fr_base_XF_fr_EpauleARG * ci.EpauleARG_Ic * xf.fr_base_XF_fr_EpauleARG';


% Contribution of link BJambeARD
ci.HJambeARD_Ic = ci.HJambeARD_Ic + xf.fr_HJambeARD_XF_fr_BJambeARD * ci.BJambeARD_Ic * xf.fr_HJambeARD_XF_fr_BJambeARD';


% Contribution of link HJambeARD
ci.EpauleARD_Ic = ci.EpauleARD_Ic + xf.fr_EpauleARD_XF_fr_HJambeARD * ci.HJambeARD_Ic * xf.fr_EpauleARD_XF_fr_HJambeARD';


% Contribution of link EpauleARD
ci.base_Ic = ci.base_Ic + xf.fr_base_XF_fr_EpauleARD * ci.EpauleARD_Ic * xf.fr_base_XF_fr_EpauleARD';


% Contribution of link BJambeAVG
ci.HJambeAVG_Ic = ci.HJambeAVG_Ic + xf.fr_HJambeAVG_XF_fr_BJambeAVG * ci.BJambeAVG_Ic * xf.fr_HJambeAVG_XF_fr_BJambeAVG';


% Contribution of link HJambeAVG
ci.EpauleAVG_Ic = ci.EpauleAVG_Ic + xf.fr_EpauleAVG_XF_fr_HJambeAVG * ci.HJambeAVG_Ic * xf.fr_EpauleAVG_XF_fr_HJambeAVG';


% Contribution of link EpauleAVG
ci.base_Ic = ci.base_Ic + xf.fr_base_XF_fr_EpauleAVG * ci.EpauleAVG_Ic * xf.fr_base_XF_fr_EpauleAVG';


% Contribution of link BJambeAVD
ci.HJambeAVD_Ic = ci.HJambeAVD_Ic + xf.fr_HJambeAVD_XF_fr_BJambeAVD * ci.BJambeAVD_Ic * xf.fr_HJambeAVD_XF_fr_BJambeAVD';


% Contribution of link HJambeAVD
ci.EpauleAVD_Ic = ci.EpauleAVD_Ic + xf.fr_EpauleAVD_XF_fr_HJambeAVD * ci.HJambeAVD_Ic * xf.fr_EpauleAVD_XF_fr_HJambeAVD';


% Contribution of link EpauleAVD
ci.base_Ic = ci.base_Ic + xf.fr_base_XF_fr_EpauleAVD * ci.EpauleAVD_Ic * xf.fr_base_XF_fr_EpauleAVD';


end
