classdef UAV
    %UAV Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        
        estimatedPosition;
        previousEstimatedPosition;
        
        estimatedHeading;
        
        % Last control command issued
        lastU;
        
        changesInSpiral;
        
        % 1 = exploring
        % 2 = limits avoidance
        state;
        
        stepsExploring;
        
        thresholdLimits;
    end
    
    methods
        
        % Constructor
        function uav = UAV()
            
            % UAVs will be launched around (0,0)
            uav.estimatedPosition = [0; 0];
            uav.previousEstimatedPosition = [0; 0];
            
            % Unknown, so 0
            uav.estimatedHeading = 0;
            
            uav.lastU = [-1; -1];
            
            uav.changesInSpiral = 0;
            
            % Exploration first
            uav.state = 1;
            
            uav.stepsExploring = 0;
            
            uav.thresholdLimits = 50;
            
        end
        
        
        % GETTER AND SETTERS
        
        function pos = getEstimatedPosition(uav)
           pos = uav.estimatedPosition;
        end
%         
%         function setLastEstimatedPosition(uav, new)
%            uav.lastEstimatedPosition = new; 
%         end
%         
        function heading = getEstimatedHeading(uav)
           heading = uav.estimatedHeading; 
        end
        
        %         
%         function setLastEstimatedHeading(uav, new)
%            uav.lastEstimatedHeading = new; 
%         end

        % --- GETTER AND SETTERS ---
        
        
        
        % Returns an estimation of the position with an accuracy of +- 3m
        function uav = simulateGPSreading(uav, realPosition)
            
            % History of 1 step back
            uav.previousEstimatedPosition = uav.estimatedPosition;
            
            % Accuracy +- 3 m is controlled with a N(0,1), as
            % 99% of its values are in (-3, 3)
            uav.estimatedPosition = realPosition(1:2) + randn(2,1);
            
        end
        
        
        % Runs the FSM of the UAV based on the current measurement and time
        function [uav, u] = simulateDecision(uav, cloudMeasurement, kk, dt)
            
            vectorFromLastEstimatedPositionToCurrent = ...
                uav.estimatedPosition - uav.previousEstimatedPosition;
            
            uav.estimatedHeading = ...
                pi/2 - atan2(vectorFromLastEstimatedPositionToCurrent(2), ...
                vectorFromLastEstimatedPositionToCurrent(1));
    
            
            switch uav.state
        
                % Exploring
                case 1,
                    
                    % Too close to limits. Limits avoidance
                    if (abs(abs(uav.estimatedPosition(1)) - 1000) < uav.thresholdLimits || ...
                            abs(abs(uav.estimatedPosition(2)) - 1000) < uav.thresholdLimits )
                        
                        uav.state = 2;
                        
                        % ORIENTATE UNTIL HEADING IN ESCAPE POSITION
                        
                        v = 0;
                        mu = 0;
                        
                    else
                    
                        uav.stepsExploring = uav.stepsExploring + 1;

                        if uav.stepsExploring < 5

                            v = 20;
                            mu = 0;

                        else

                            v = 20;

                            if(mod(uav.stepsExploring, uav.changesInSpiral*uav.changesInSpiral/2 + 1) == 0)

                                uav.changesInSpiral = uav.changesInSpiral + 1;
                                mu = max(0.002, 0.1 - log(uav.changesInSpiral)*0.005);

                            else

                                mu = uav.lastU(2);

                            end

                        end
                    end
                    

                    % From 0.1 to 0.002, which is equals to
                    % from e^-2.3026 to e^-6.2146
                    %mu = 2.71828^(min(-2.3026, deg2rad(-2.3026 - stepsExploring*0.1)));
                    %mu = max(0.002, 0.1 - log(uav.stepsExploring)*0.005);
                    %mu = 0.1
                    
                    
                % Limits avoidance
                case 2,
                    
                    v = 0;
                    mu = 0;
                    
            end
            
            uav.lastU = [v; mu];

            % Control command with some noise
            u = [v + 0.01*randn(); mu + 0.0001*randn()];
    
        end
        
    end
    
end

