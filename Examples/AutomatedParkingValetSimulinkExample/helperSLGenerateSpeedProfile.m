function speedProfile = helperSLGenerateSpeedProfile(splineData, speedConfig)

%helperSLGenerateSpeedProfile generate a speed profile for the reference path.

% Copyright 2017-2018 The MathWorks, Inc.

persistent speedGenerator

if isempty(speedGenerator)
    speedGenerator = HelperSpeedProfileGenerator(splineData);
    speedGenerator = helperSLMatchStructFields(speedGenerator, speedConfig);
else
    speedGenerator = helperSLMatchStructFields(speedGenerator, speedConfig);
    speedGenerator.setReferencePath(splineData)
end

% Generate speed profile
speedProfile = generate(speedGenerator);
