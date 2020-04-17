%% Possibly fix the paths according to your configuration

[rc spv2] = system('locate -b spatial_v2'); % does not work in every OS
if( rc != 0 )
    error('Could not find spatial_v2 on the file system. Aborting');
end
spv2 = spv2(1:end-1); % get rid of garbage

addpath(genpath( spv2 ) );                % The root of 'spatial_v2'
addpath('../octave');  % The robot-specific Octave generated code
addpath('../models');  % The generated model in Roy's format (also Octave)

% Add to the path the Octave-tests root
%
if( ! exist('octave-tests', 'dir') )
    error('Please make a symbolic link to the "octave-tests/" subfolder of the RobCoGen root');
end
addpath('octave-tests/src/');
addpath('octave-tests/src/cpp');

%% These paths were automatically generated! Check them!
exe_id = './build/id';
exe_fd = './build/fd';
exe_im = './build/jsim';

me.kk = modelConstants();
me.ip = inertiaProperties(me.kk);
me.xm = initMotionTransforms(me.kk);
me.xf = initForceTransforms(me.kk);

roy.model = popi(me.kk);
