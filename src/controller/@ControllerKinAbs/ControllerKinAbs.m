classdef ControllerKinAbs < mystica.controller.Base

    properties (SetAccess=protected,GetAccess=public)
        mBodyTwist_0
    end

    methods
        function obj = ControllerKinAbs(input)
            arguments
                input.model
                input.stateKinMBody
                input.stgsController
                input.stgsDesiredShape
                input.time
                input.controller_dt
            end

            obj@mystica.controller.Base(...
                'model',input.model,...
                'state',input.stateKinMBody,...
                'stgsController',input.stgsController,...
                'stgsDesiredShape',input.stgsDesiredShape,...
                'time',input.time,...
                'controller_dt',input.controller_dt);

            obj.computeCostFunctionAndConstraints('model',input.model,'stateKinMBody',input.stateKinMBody);
            obj.mBodyTwist_0 = zeros(input.model.constants.mBodyTwist,1);

        end

        function mBodyTwist_0 = solve(obj,input)
            arguments
                obj
                input.time
                input.stateKinMBody
                input.model
            end
            obj.opti.set_initial(obj.csdSy.mBodyTwist_0,obj.mBodyTwist_0);
            obj.opti.set_value(obj.csdSy.mBodyPosQuat_0,input.stateKinMBody.mBodyPosQuat_0);
            obj.opti.set_value(obj.csdSy.time,input.time);
            obj.opti.set_value(obj.csdSy.mBodyTwist_0_lastSol,obj.mBodyTwist_0);
            try
                sol = obj.opti.solve();
                obj.mBodyTwist_0 = sol.value(obj.csdSy.mBodyTwist_0);
            catch
                warning('QP failed')
            end
            mBodyTwist_0 = obj.mBodyTwist_0;
        end

    end
    methods (Access=protected)
        computeCostFunctionAndConstraints(obj)
    end
end
