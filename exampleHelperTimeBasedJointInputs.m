function stateDot = exampleHelperTimeBasedJointInputs(motionModel, timeInterval, configWaypoints, t, state)
% This function is for internal use only and may be removed in a future
% release

%exampleHelperTimeBasedJointInputs Pass time-varying inputs to the jointSpaceMotionModel derivative
%   Since the jointSpaceMotionModel derivative method is updated at an
%   instant in time, a wrapper function is needed to provide time-varying
%   tracking inputs. This function computes the value of the B-spline
%   trajectory at an instant in time, t, and provides that input to the
%   derivative of the associated jointSpaceMotionModel at that same
%   instant. The resultant state derivative can be passed to an ODE solver
%   to compute the tracking behavior.

% Copyright 2019 The MathWorks, Inc.

    % Use a B-spline curve to ensure the trajectory is smooth and moves
    % through the waypoints with non-zero velocity
    [qd, qdDot] = bsplinepolytraj(configWaypoints,  timeInterval , t);
    
    % Compute state derivative
    stateDot = derivative(motionModel, state, [qd; qdDot]);
end