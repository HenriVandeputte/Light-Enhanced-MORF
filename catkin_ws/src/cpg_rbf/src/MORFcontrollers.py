import numpy as np
from ControllerBlocks import CPG, VRN, InputPreprocessor, MotorNeurons, RBFN, Relu


# this file contains the controllers, along with an example of an empty controller for future controllers

class ControllerExample:
    def __init__(self, Robot):
        self.Robot = Robot
        self.placeholder = None
        # define member variables and classes
        pass

    def reset(self):
        # function that resets controller to initial setup
        pass

    def get_params(self):
        # return params for learning
        return self.placeholder

    def set_params(self, params):
        # set params (inverse of self.getParams)
        return self.placeholder

    def run(self):
        # run controller and set_target_config on Robot
        config = None
        self.Robot.set_target_config(config)


class RBFNNControl:
    def __init__(self, Robot, motorMapping="indirect", legsDisabled=None, clipJointValues=False):
        self.Robot = Robot
        self.cpg = CPG()

        if legsDisabled is None:
            self.legsDisabled = [False] * 6
        else:
            self.legsDisabled = legsDisabled

        self.disabledConfig = [0, 3.0, -2.0]
        self.clipJointValues = clipJointValues

        self.cpg.set_frequency()
        self.vrn = VRN()
        self.rbfn = RBFN(2, 20, 18)

        self.motorMapper = MotorMappings(motorMapping)

    def reset(self):
        self.Robot = self.Robot
        self.cpg = CPG()

        self.legsDisabled = self.legsDisabled

        self.disabledConfig = [0, 3.0, -2.0]

        self.cpg.set_frequency()
        self.vrn = VRN()
        self.rbfn = RBFN(2, 20, 18)

    def reset_CPG(self):
        self.cpg = CPG()
        self.cpg.set_frequency()

    def get_params(self):

        return self.rbfn.get_weights()

    def set_params(self, params):
        self.rbfn.set_weights(params)

    def set_disabled_legs(self, bools):
        self.legsDisabled = bools

    def lock_disabled_legs(self, config):
        locked_config = config
        for i, disabled in enumerate(self.legsDisabled):
            if disabled:
                locked_config[i * 3:i * 3 + 3] = self.disabledConfig
        return locked_config

    def run(self):

        output_h2, output_h1 = self.cpg.update()
        rbfn_out = self.rbfn.forward([output_h1, output_h2])

        if self.clipJointValues == "true":

            joints_range = np.array([0.35, 1.0, 0.6]*6)

            rbfn_out = np.maximum(np.minimum(rbfn_out, joints_range), -joints_range)

        if np.isnan(rbfn_out).any():
            exit("nan encountered in controller output")

        config = self.motorMapper.map(rbfn_out)
        config = self.lock_disabled_legs(config)
        self.Robot.set_target_config(config)

        

class Motormapping_angle():
    def __init__(self):
        self.input = [-0.2,0.2]

        self.output_tjoint = [1800,2200]
        self.output_cjoint = [2500,3000]
        self.output_fjoint = [1500,2000]

        # self.output_tjoint = [1800,2300]
        # self.output_cjoint = [2000,2500]
        # self.output_fjoint = [1000,1500]

        # slope = (output_end - output_start) / (input_end - input_start)
        # output = output_start + slope * (input - input_start)

        self.slope_tjoint = 1.0 * (self.output_tjoint[1] - self.output_tjoint[0]) / (self.input[1] - self.input[0])
        self.slope_cjoint = 1.0 * (self.output_cjoint[1] - self.output_cjoint[0]) / (self.input[1] - self.input[0])
        self.slope_fjoint = 1.0 * (self.output_fjoint[1] - self.output_fjoint[0]) / (self.input[1] - self.input[0])

        
    def map(self,control_input):

        leg0 = [ int(self.output_tjoint[0] + self.slope_tjoint * (control_input[0]*control_input[3] - self.input[0])), 
                int(self.output_cjoint[1] + self.slope_cjoint * (Relu(-control_input[1]) - self.input[0])),
                int(self.output_fjoint[0] + self.slope_fjoint * (Relu(control_input[2]) - self.input[0]))]

        leg1 = [ int(self.output_tjoint[0] + self.slope_tjoint * (-control_input[0]*control_input[3] - self.input[0])), 
                int(self.output_cjoint[1] + self.slope_cjoint * (Relu(control_input[1]) - self.input[0])),
                int(self.output_fjoint[0] + self.slope_fjoint * (Relu(-control_input[2]) - self.input[0]))]

        leg2 = [ int(self.output_tjoint[0] + self.slope_tjoint * (control_input[0]*control_input[3] - self.input[0])), 
                int(self.output_cjoint[1] + self.slope_cjoint * (Relu(-control_input[1]) - self.input[0])),
                int(self.output_fjoint[0] + self.slope_fjoint * (Relu(control_input[2]) - self.input[0]))]

        leg3 = [ int(self.output_tjoint[0] + self.slope_tjoint * (control_input[0]*control_input[4] - self.input[0])), 
                int(self.output_cjoint[1] + self.slope_cjoint * (Relu(control_input[1]) - self.input[0])),
                int(self.output_fjoint[0] + self.slope_fjoint * (Relu(-control_input[2]) - self.input[0]))]

        leg4 = [ int(self.output_tjoint[0] + self.slope_tjoint * (-control_input[0]*control_input[4] - self.input[0])), 
                int(self.output_cjoint[1] + self.slope_cjoint * (Relu(-control_input[1]) - self.input[0])),
                int(self.output_fjoint[0] + self.slope_fjoint * (Relu(control_input[2]) - self.input[0]))]

        leg5 = [ int(self.output_tjoint[0] + self.slope_tjoint * (control_input[0]*control_input[4] - self.input[0])), 
                int(self.output_cjoint[1] + self.slope_cjoint * (Relu(control_input[1])- self.input[0])),
                int(self.output_fjoint[0] + self.slope_fjoint * (Relu(-control_input[2]) - self.input[0]))]

        return leg0 + leg1 + leg2 + leg3 + leg4 + leg5


class MotorMappings:
    def __init__(self, mapping):
        self.bias_tjoint = 0.0
        self.bias_cjoint = np.pi / 2
        self.bias_fjoint = -np.pi / 4

        self.biasVec = np.array([self.bias_tjoint, self.bias_cjoint, self.bias_fjoint] * 6)

        self.mapping = {"indirect": self.indirect_mapping, "semiIndirect": self.semi_indirect_mapping,
                        "direct": self.direct_mapping}[mapping]

    def indirect_mapping(self, control_input):
        leg0 = [control_input[0] + self.bias_tjoint, -control_input[1] + self.bias_cjoint,
                control_input[2] + self.bias_fjoint]
        leg1 = [-control_input[0] + self.bias_tjoint, control_input[1] + self.bias_cjoint,
                -control_input[2] + self.bias_fjoint]
        leg2 = [control_input[0] + self.bias_tjoint, -control_input[1] + self.bias_cjoint,
                control_input[2] + self.bias_fjoint]
        leg3 = [control_input[0] + self.bias_tjoint, control_input[1] + self.bias_cjoint,
                -control_input[2] + self.bias_fjoint]
        leg4 = [-control_input[0] + self.bias_tjoint, -control_input[1] + self.bias_cjoint,
                control_input[2] + self.bias_fjoint]
        leg5 = [control_input[0] + self.bias_tjoint, control_input[1] + self.bias_cjoint,
                -control_input[2] + self.bias_fjoint]

        return leg0 + leg1 + leg2 + leg3 + leg4 + leg5

    def semi_indirect_mapping(self, control_input):
        leg0 = [control_input[0] + self.bias_tjoint, -control_input[1] + self.bias_cjoint,
                control_input[2] + self.bias_fjoint]
        leg1 = [-control_input[3] + self.bias_tjoint, control_input[4] + self.bias_cjoint,
                -control_input[5] + self.bias_fjoint]
        leg2 = [control_input[6] + self.bias_tjoint, -control_input[7] + self.bias_cjoint,
                control_input[8] + self.bias_fjoint]
        leg3 = [control_input[0] + self.bias_tjoint, control_input[1] + self.bias_cjoint,
                -control_input[2] + self.bias_fjoint]
        leg4 = [-control_input[3] + self.bias_tjoint, -control_input[4] + self.bias_cjoint,
                control_input[5] + self.bias_fjoint]
        leg5 = [control_input[6] + self.bias_tjoint, control_input[7] + self.bias_cjoint,
                -control_input[8] + self.bias_fjoint]

        return leg0 + leg1 + leg2 + leg3 + leg4 + leg5

    def direct_mapping(self, control_input):
        leg0 = [control_input[0] + self.bias_tjoint, -control_input[1] + self.bias_cjoint,
                control_input[2] + self.bias_fjoint]
        leg1 = [-control_input[3] + self.bias_tjoint, control_input[4] + self.bias_cjoint,
                -control_input[5] + self.bias_fjoint]
        leg2 = [control_input[6] + self.bias_tjoint, -control_input[7] + self.bias_cjoint,
                control_input[8] + self.bias_fjoint]
        leg3 = [control_input[9] + self.bias_tjoint, control_input[10] + self.bias_cjoint,
                -control_input[11] + self.bias_fjoint]
        leg4 = [-control_input[12] + self.bias_tjoint, -control_input[13] + self.bias_cjoint,
                control_input[14] + self.bias_fjoint]
        leg5 = [control_input[15] + self.bias_tjoint, control_input[16] + self.bias_cjoint,
                -control_input[17] + self.bias_fjoint]

        return leg0 + leg1 + leg2 + leg3 + leg4 + leg5

    def map(self, control_input):
        return self.mapping(control_input)
