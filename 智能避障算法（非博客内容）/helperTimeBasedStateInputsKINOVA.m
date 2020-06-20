function stateDot = helperTimeBasedStateInputsKINOVA(obj, timeInterval, jointStates, t, state)
    % Copyright 2019 The MathWorks, Inc.

    targetState = interp1(timeInterval, jointStates, t); % %bsplinepolytraj(taskJWaypoints',  timeInterval , t);
    
    % Compute state derivative
    stateDot = derivative(obj, state, targetState);
end
