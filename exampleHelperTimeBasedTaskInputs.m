function stateDot = exampleHelperTimeBasedTaskInputs(motionModel, timeInterval, initialTform, finalTform, t, state)
% This function is for internal use only and may be removed in a future
% release

%exampleHelperTimeBasedTaskInputs Pass time-varying inputs to the taskSpaceMotionModel derivative
%   Since the taskSpaceMotionModel derivative method is updated at an
%   instant in time, a wrapper function is needed to provide time-varying
%   tracking inputs. This function computes the value of the transform
%   trajectory at an instant in time, t, and provides that input to the
%   derivative of the associated taskSpaceMotionModel at that same instant.
%   The resultant state derivative can be passed to an ODE solver to
%   compute the tracking behavior.

% Copyright 2019 The MathWorks, Inc.

[refPose, refVel] = transformtraj(initialTform, finalTform, timeInterval, t);

stateDot = derivative(motionModel, state, refPose, refVel);

end