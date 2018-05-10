classdef CountTime < matlab.System & matlab.system.mixin.SampleTime
    % Counts Hits and Time
    
    properties(Nontunable)
        SampleTimeTypeProp = 'Discrete sample time'; % Sample Time Type
        SampleTime = 1.4; % Sample Time
        OffsetTime = 0.2; % Offset Time
        TickTime = 0.1;
    end
    
    properties(DiscreteState)
        Count
    end
    
    properties(Constant, Hidden)
        SampleTimeTypePropSet = matlab.system.StringSet(...
           {'Inherited sample time',...
            'Inherited not controllable sample time',...
            'Fixed In Minor Step sample time', ...
            'Discrete sample time', ...
            'Controllable sample time'});
    end
    methods(Access = protected)
        function sts = getSampleTimeImpl(obj)
            switch obj.SampleTimeTypeProp
                case 'Inherited sample time'
                    sts = createSampleTime(obj,'Type','Inherited');
                case 'Inherited not controllable sample time'
                    sts = createSampleTime(obj,'Type','Inherited',...
                        'Disallow','Controllable');
                case 'Fixed In Minor Step sample time'
                    sts = createSampleTime(obj,'Type','Fixed In Minor Step');
                case 'Discrete sample time'
                    sts = createSampleTime(obj,'Type','Discrete',...
                      'SampleTime',obj.SampleTime, ...
                      'OffsetTime',obj.OffsetTime);
                case 'Controllable sample time'
                    sts = createSampleTime(obj,'Type','Controllable',...
                        'TickTime',obj.TickTime);
            end
        end
        
        function [Count, Time, SampleTime] = stepImpl(obj,u)
           Count = obj.Count + u;
            obj.Count = Count;
            Time = getCurrentTime(obj);
            sts = getSampleTime(obj);
            if strcmp(sts.Type,'Controllable')
               setNumTicksUntilNextHit(obj,obj.Count);
            end
            SampleTime = sts.SampleTime;
        end
        
        function setupImpl(obj)
            obj.Count = 0;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.Count = 0;
        end
        
        function flag = isInactivePropertyImpl(obj,prop)
            flag = false;
            switch obj.SampleTimeTypeProp
                case {'Inherited sample time', ...
                        'Inherited not controllable sample time', ...
                        'Fixed In Minor Step sample time'}
                    if any(strcmp(prop,{'SampleTime','OffsetTime','TickTime'}))
                        flag = true;
                    end
                case 'discrete'
                    if any(strcmp(prop,{'TickTime'}))
                        flag = true;
                    end
                case 'controllable'
                    if any(strcmp(prop,{'SampleTime','OffsetTime'}))
                        flag = true;
                    end
            end
        end
    end
end