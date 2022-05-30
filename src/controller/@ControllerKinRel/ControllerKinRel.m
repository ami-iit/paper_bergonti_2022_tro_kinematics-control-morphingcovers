classdef ControllerKinRel < mystica.controller.Base

    properties (SetAccess=protected,GetAccess=public)
        motorsAngVel
    end

    methods
        function obj = ControllerKinRel(input)
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
            obj.motorsAngVel = zeros(input.model.constants.motorsAngVel,1);

        end

        function motorsAngVel = solve(obj,input)
            arguments
                obj
                input.time
                input.stateKinMBody
                input.model
            end

            Zact    = input.stateKinMBody.getZact('model',input.model);
            invZact = mystica.utils.pinvDamped(Zact,obj.stgsController.regTermDampPInv);

            obj.opti.set_initial(obj.csdSy.motorsAngVel,obj.motorsAngVel);
            obj.opti.set_value(obj.csdSy.mBodyPosQuat_0,input.stateKinMBody.mBodyPosQuat_0);
            obj.opti.set_value(obj.csdSy.time,input.time);
            obj.opti.set_value(obj.csdSy.motorsAngVel_lastSol,obj.motorsAngVel);
            obj.opti.set_value(obj.csdSy.nullJc_mBodyTwist_0,input.stateKinMBody.nullJc_mBodyTwist_0);
            obj.opti.set_value(obj.csdSy.invZact,invZact);
            try
                sol = obj.opti.solve();
                obj.motorsAngVel = sol.value(obj.csdSy.motorsAngVel);
            catch
                warning('QP failed')
                obj.motorsAngVel = obj.motorsAngVel*0;
            end
            motorsAngVel = obj.motorsAngVel;

        end

    end

    methods (Access=protected)
        computeCostFunctionAndConstraints(obj)
    end
end
