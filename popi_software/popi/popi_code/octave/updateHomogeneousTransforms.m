function out = updateHomogeneousTransforms(tr, q, params, consts)

sin_q_rf_haa_joint = sin( q(1));
cos_q_rf_haa_joint = cos( q(1));
tr.fr_EpauleAVD_Xh_fr_base(1,2) = sin_q_rf_haa_joint;
tr.fr_EpauleAVD_Xh_fr_base(1,3) = cos_q_rf_haa_joint;
tr.fr_EpauleAVD_Xh_fr_base(1,4) = (- consts.ty_rf_haa_joint * sin_q_rf_haa_joint)-( consts.tz_rf_haa_joint * cos_q_rf_haa_joint);
tr.fr_EpauleAVD_Xh_fr_base(2,2) = cos_q_rf_haa_joint;
tr.fr_EpauleAVD_Xh_fr_base(2,3) = -sin_q_rf_haa_joint;
tr.fr_EpauleAVD_Xh_fr_base(2,4) = ( consts.tz_rf_haa_joint * sin_q_rf_haa_joint)-( consts.ty_rf_haa_joint * cos_q_rf_haa_joint);

sin_q_rf_haa_joint = sin( q(1));
cos_q_rf_haa_joint = cos( q(1));
tr.fr_base_Xh_fr_EpauleAVD(2,1) = sin_q_rf_haa_joint;
tr.fr_base_Xh_fr_EpauleAVD(2,2) = cos_q_rf_haa_joint;
tr.fr_base_Xh_fr_EpauleAVD(3,1) = cos_q_rf_haa_joint;
tr.fr_base_Xh_fr_EpauleAVD(3,2) = -sin_q_rf_haa_joint;

sin_q_rf_hfe_joint = sin( q(2));
cos_q_rf_hfe_joint = cos( q(2));
tr.fr_HJambeAVD_Xh_fr_EpauleAVD(1,1) = sin_q_rf_hfe_joint;
tr.fr_HJambeAVD_Xh_fr_EpauleAVD(1,3) = -cos_q_rf_hfe_joint;
tr.fr_HJambeAVD_Xh_fr_EpauleAVD(2,1) = cos_q_rf_hfe_joint;
tr.fr_HJambeAVD_Xh_fr_EpauleAVD(2,3) = sin_q_rf_hfe_joint;

sin_q_rf_hfe_joint = sin( q(2));
cos_q_rf_hfe_joint = cos( q(2));
tr.fr_EpauleAVD_Xh_fr_HJambeAVD(1,1) = sin_q_rf_hfe_joint;
tr.fr_EpauleAVD_Xh_fr_HJambeAVD(1,2) = cos_q_rf_hfe_joint;
tr.fr_EpauleAVD_Xh_fr_HJambeAVD(3,1) = -cos_q_rf_hfe_joint;
tr.fr_EpauleAVD_Xh_fr_HJambeAVD(3,2) = sin_q_rf_hfe_joint;

sin_q_rf_kfe_joint = sin( q(3));
cos_q_rf_kfe_joint = cos( q(3));
tr.fr_BJambeAVD_Xh_fr_HJambeAVD(1,1) = cos_q_rf_kfe_joint;
tr.fr_BJambeAVD_Xh_fr_HJambeAVD(1,2) = -sin_q_rf_kfe_joint;
tr.fr_BJambeAVD_Xh_fr_HJambeAVD(1,4) = ( consts.ty_rf_kfe_joint * sin_q_rf_kfe_joint)-( consts.tx_rf_kfe_joint * cos_q_rf_kfe_joint);
tr.fr_BJambeAVD_Xh_fr_HJambeAVD(2,1) = -sin_q_rf_kfe_joint;
tr.fr_BJambeAVD_Xh_fr_HJambeAVD(2,2) = -cos_q_rf_kfe_joint;
tr.fr_BJambeAVD_Xh_fr_HJambeAVD(2,4) = ( consts.tx_rf_kfe_joint * sin_q_rf_kfe_joint)+( consts.ty_rf_kfe_joint * cos_q_rf_kfe_joint);

sin_q_rf_kfe_joint = sin( q(3));
cos_q_rf_kfe_joint = cos( q(3));
tr.fr_HJambeAVD_Xh_fr_BJambeAVD(1,1) = cos_q_rf_kfe_joint;
tr.fr_HJambeAVD_Xh_fr_BJambeAVD(1,2) = -sin_q_rf_kfe_joint;
tr.fr_HJambeAVD_Xh_fr_BJambeAVD(2,1) = -sin_q_rf_kfe_joint;
tr.fr_HJambeAVD_Xh_fr_BJambeAVD(2,2) = -cos_q_rf_kfe_joint;

sin_q_lf_haa_joint = sin( q(4));
cos_q_lf_haa_joint = cos( q(4));
tr.fr_EpauleAVG_Xh_fr_base(1,2) = sin_q_lf_haa_joint;
tr.fr_EpauleAVG_Xh_fr_base(1,3) = -cos_q_lf_haa_joint;
tr.fr_EpauleAVG_Xh_fr_base(1,4) = ( consts.tz_lf_haa_joint * cos_q_lf_haa_joint)-( consts.ty_lf_haa_joint * sin_q_lf_haa_joint);
tr.fr_EpauleAVG_Xh_fr_base(2,2) = cos_q_lf_haa_joint;
tr.fr_EpauleAVG_Xh_fr_base(2,3) = sin_q_lf_haa_joint;
tr.fr_EpauleAVG_Xh_fr_base(2,4) = (- consts.tz_lf_haa_joint * sin_q_lf_haa_joint)-( consts.ty_lf_haa_joint * cos_q_lf_haa_joint);

sin_q_lf_haa_joint = sin( q(4));
cos_q_lf_haa_joint = cos( q(4));
tr.fr_base_Xh_fr_EpauleAVG(2,1) = sin_q_lf_haa_joint;
tr.fr_base_Xh_fr_EpauleAVG(2,2) = cos_q_lf_haa_joint;
tr.fr_base_Xh_fr_EpauleAVG(3,1) = -cos_q_lf_haa_joint;
tr.fr_base_Xh_fr_EpauleAVG(3,2) = sin_q_lf_haa_joint;

sin_q_lf_hfe_joint = sin( q(5));
cos_q_lf_hfe_joint = cos( q(5));
tr.fr_HJambeAVG_Xh_fr_EpauleAVG(1,1) = -sin_q_lf_hfe_joint;
tr.fr_HJambeAVG_Xh_fr_EpauleAVG(1,3) = cos_q_lf_hfe_joint;
tr.fr_HJambeAVG_Xh_fr_EpauleAVG(2,1) = -cos_q_lf_hfe_joint;
tr.fr_HJambeAVG_Xh_fr_EpauleAVG(2,3) = -sin_q_lf_hfe_joint;

sin_q_lf_hfe_joint = sin( q(5));
cos_q_lf_hfe_joint = cos( q(5));
tr.fr_EpauleAVG_Xh_fr_HJambeAVG(1,1) = -sin_q_lf_hfe_joint;
tr.fr_EpauleAVG_Xh_fr_HJambeAVG(1,2) = -cos_q_lf_hfe_joint;
tr.fr_EpauleAVG_Xh_fr_HJambeAVG(3,1) = cos_q_lf_hfe_joint;
tr.fr_EpauleAVG_Xh_fr_HJambeAVG(3,2) = -sin_q_lf_hfe_joint;

sin_q_lf_kfe_joint = sin( q(6));
cos_q_lf_kfe_joint = cos( q(6));
tr.fr_BJambeAVG_Xh_fr_HJambeAVG(1,1) = cos_q_lf_kfe_joint;
tr.fr_BJambeAVG_Xh_fr_HJambeAVG(1,2) = -sin_q_lf_kfe_joint;
tr.fr_BJambeAVG_Xh_fr_HJambeAVG(1,4) = ( consts.ty_lf_kfe_joint * sin_q_lf_kfe_joint)-( consts.tx_lf_kfe_joint * cos_q_lf_kfe_joint);
tr.fr_BJambeAVG_Xh_fr_HJambeAVG(2,1) = -sin_q_lf_kfe_joint;
tr.fr_BJambeAVG_Xh_fr_HJambeAVG(2,2) = -cos_q_lf_kfe_joint;
tr.fr_BJambeAVG_Xh_fr_HJambeAVG(2,4) = ( consts.tx_lf_kfe_joint * sin_q_lf_kfe_joint)+( consts.ty_lf_kfe_joint * cos_q_lf_kfe_joint);

sin_q_lf_kfe_joint = sin( q(6));
cos_q_lf_kfe_joint = cos( q(6));
tr.fr_HJambeAVG_Xh_fr_BJambeAVG(1,1) = cos_q_lf_kfe_joint;
tr.fr_HJambeAVG_Xh_fr_BJambeAVG(1,2) = -sin_q_lf_kfe_joint;
tr.fr_HJambeAVG_Xh_fr_BJambeAVG(2,1) = -sin_q_lf_kfe_joint;
tr.fr_HJambeAVG_Xh_fr_BJambeAVG(2,2) = -cos_q_lf_kfe_joint;

sin_q_rh_haa_joint = sin( q(7));
cos_q_rh_haa_joint = cos( q(7));
tr.fr_EpauleARD_Xh_fr_base(1,2) = sin_q_rh_haa_joint;
tr.fr_EpauleARD_Xh_fr_base(1,3) = cos_q_rh_haa_joint;
tr.fr_EpauleARD_Xh_fr_base(1,4) = (- consts.ty_rh_haa_joint * sin_q_rh_haa_joint)-( consts.tz_rh_haa_joint * cos_q_rh_haa_joint);
tr.fr_EpauleARD_Xh_fr_base(2,2) = cos_q_rh_haa_joint;
tr.fr_EpauleARD_Xh_fr_base(2,3) = -sin_q_rh_haa_joint;
tr.fr_EpauleARD_Xh_fr_base(2,4) = ( consts.tz_rh_haa_joint * sin_q_rh_haa_joint)-( consts.ty_rh_haa_joint * cos_q_rh_haa_joint);

sin_q_rh_haa_joint = sin( q(7));
cos_q_rh_haa_joint = cos( q(7));
tr.fr_base_Xh_fr_EpauleARD(2,1) = sin_q_rh_haa_joint;
tr.fr_base_Xh_fr_EpauleARD(2,2) = cos_q_rh_haa_joint;
tr.fr_base_Xh_fr_EpauleARD(3,1) = cos_q_rh_haa_joint;
tr.fr_base_Xh_fr_EpauleARD(3,2) = -sin_q_rh_haa_joint;

sin_q_rh_hfe_joint = sin( q(8));
cos_q_rh_hfe_joint = cos( q(8));
tr.fr_HJambeARD_Xh_fr_EpauleARD(1,1) = sin_q_rh_hfe_joint;
tr.fr_HJambeARD_Xh_fr_EpauleARD(1,3) = -cos_q_rh_hfe_joint;
tr.fr_HJambeARD_Xh_fr_EpauleARD(2,1) = cos_q_rh_hfe_joint;
tr.fr_HJambeARD_Xh_fr_EpauleARD(2,3) = sin_q_rh_hfe_joint;

sin_q_rh_hfe_joint = sin( q(8));
cos_q_rh_hfe_joint = cos( q(8));
tr.fr_EpauleARD_Xh_fr_HJambeARD(1,1) = sin_q_rh_hfe_joint;
tr.fr_EpauleARD_Xh_fr_HJambeARD(1,2) = cos_q_rh_hfe_joint;
tr.fr_EpauleARD_Xh_fr_HJambeARD(3,1) = -cos_q_rh_hfe_joint;
tr.fr_EpauleARD_Xh_fr_HJambeARD(3,2) = sin_q_rh_hfe_joint;

sin_q_rh_kfe_joint = sin( q(9));
cos_q_rh_kfe_joint = cos( q(9));
tr.fr_BJambeARD_Xh_fr_HJambeARD(1,1) = cos_q_rh_kfe_joint;
tr.fr_BJambeARD_Xh_fr_HJambeARD(1,2) = -sin_q_rh_kfe_joint;
tr.fr_BJambeARD_Xh_fr_HJambeARD(1,4) = ( consts.ty_rh_kfe_joint * sin_q_rh_kfe_joint)-( consts.tx_rh_kfe_joint * cos_q_rh_kfe_joint);
tr.fr_BJambeARD_Xh_fr_HJambeARD(2,1) = -sin_q_rh_kfe_joint;
tr.fr_BJambeARD_Xh_fr_HJambeARD(2,2) = -cos_q_rh_kfe_joint;
tr.fr_BJambeARD_Xh_fr_HJambeARD(2,4) = ( consts.tx_rh_kfe_joint * sin_q_rh_kfe_joint)+( consts.ty_rh_kfe_joint * cos_q_rh_kfe_joint);

sin_q_rh_kfe_joint = sin( q(9));
cos_q_rh_kfe_joint = cos( q(9));
tr.fr_HJambeARD_Xh_fr_BJambeARD(1,1) = cos_q_rh_kfe_joint;
tr.fr_HJambeARD_Xh_fr_BJambeARD(1,2) = -sin_q_rh_kfe_joint;
tr.fr_HJambeARD_Xh_fr_BJambeARD(2,1) = -sin_q_rh_kfe_joint;
tr.fr_HJambeARD_Xh_fr_BJambeARD(2,2) = -cos_q_rh_kfe_joint;

sin_q_lh_haa_joint = sin( q(10));
cos_q_lh_haa_joint = cos( q(10));
tr.fr_EpauleARG_Xh_fr_base(1,2) = sin_q_lh_haa_joint;
tr.fr_EpauleARG_Xh_fr_base(1,3) = -cos_q_lh_haa_joint;
tr.fr_EpauleARG_Xh_fr_base(1,4) = ( consts.tz_lh_haa_joint * cos_q_lh_haa_joint)-( consts.ty_lh_haa_joint * sin_q_lh_haa_joint);
tr.fr_EpauleARG_Xh_fr_base(2,2) = cos_q_lh_haa_joint;
tr.fr_EpauleARG_Xh_fr_base(2,3) = sin_q_lh_haa_joint;
tr.fr_EpauleARG_Xh_fr_base(2,4) = (- consts.tz_lh_haa_joint * sin_q_lh_haa_joint)-( consts.ty_lh_haa_joint * cos_q_lh_haa_joint);

sin_q_lh_haa_joint = sin( q(10));
cos_q_lh_haa_joint = cos( q(10));
tr.fr_base_Xh_fr_EpauleARG(2,1) = sin_q_lh_haa_joint;
tr.fr_base_Xh_fr_EpauleARG(2,2) = cos_q_lh_haa_joint;
tr.fr_base_Xh_fr_EpauleARG(3,1) = -cos_q_lh_haa_joint;
tr.fr_base_Xh_fr_EpauleARG(3,2) = sin_q_lh_haa_joint;

sin_q_lh_hfe_joint = sin( q(11));
cos_q_lh_hfe_joint = cos( q(11));
tr.fr_HJambeARG_Xh_fr_EpauleARG(1,1) = -sin_q_lh_hfe_joint;
tr.fr_HJambeARG_Xh_fr_EpauleARG(1,3) = cos_q_lh_hfe_joint;
tr.fr_HJambeARG_Xh_fr_EpauleARG(2,1) = -cos_q_lh_hfe_joint;
tr.fr_HJambeARG_Xh_fr_EpauleARG(2,3) = -sin_q_lh_hfe_joint;

sin_q_lh_hfe_joint = sin( q(11));
cos_q_lh_hfe_joint = cos( q(11));
tr.fr_EpauleARG_Xh_fr_HJambeARG(1,1) = -sin_q_lh_hfe_joint;
tr.fr_EpauleARG_Xh_fr_HJambeARG(1,2) = -cos_q_lh_hfe_joint;
tr.fr_EpauleARG_Xh_fr_HJambeARG(3,1) = cos_q_lh_hfe_joint;
tr.fr_EpauleARG_Xh_fr_HJambeARG(3,2) = -sin_q_lh_hfe_joint;

sin_q_lh_kfe_joint = sin( q(12));
cos_q_lh_kfe_joint = cos( q(12));
tr.fr_BJambeARG_Xh_fr_HJambeARG(1,1) = cos_q_lh_kfe_joint;
tr.fr_BJambeARG_Xh_fr_HJambeARG(1,2) = -sin_q_lh_kfe_joint;
tr.fr_BJambeARG_Xh_fr_HJambeARG(1,4) = ( consts.ty_lh_kfe_joint * sin_q_lh_kfe_joint)-( consts.tx_lh_kfe_joint * cos_q_lh_kfe_joint);
tr.fr_BJambeARG_Xh_fr_HJambeARG(2,1) = -sin_q_lh_kfe_joint;
tr.fr_BJambeARG_Xh_fr_HJambeARG(2,2) = -cos_q_lh_kfe_joint;
tr.fr_BJambeARG_Xh_fr_HJambeARG(2,4) = ( consts.tx_lh_kfe_joint * sin_q_lh_kfe_joint)+( consts.ty_lh_kfe_joint * cos_q_lh_kfe_joint);

sin_q_lh_kfe_joint = sin( q(12));
cos_q_lh_kfe_joint = cos( q(12));
tr.fr_HJambeARG_Xh_fr_BJambeARG(1,1) = cos_q_lh_kfe_joint;
tr.fr_HJambeARG_Xh_fr_BJambeARG(1,2) = -sin_q_lh_kfe_joint;
tr.fr_HJambeARG_Xh_fr_BJambeARG(2,1) = -sin_q_lh_kfe_joint;
tr.fr_HJambeARG_Xh_fr_BJambeARG(2,2) = -cos_q_lh_kfe_joint;



out = tr;