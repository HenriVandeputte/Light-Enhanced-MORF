import matplotlib.pyplot as plt
import numpy as np


def tanh(a):
    return (np.exp(2 * a) - 1) / (np.exp(2 * a) + 1)

def Relu(input):
  if input > 0:
    return input
  else:
    return 0

class Delay:
  def __init__(self):
    self.w = 1
    self.output = 0
    self.number = 1

  def set_w(self,input,number):
    self.w = [input]*number
    self.output_before = [0]*number
    self.output = [0]*number

  def update(self,input):
    self.output[0] = np.tanh(input+(self.w[0]*self.output[0]))
    for i in range(len(self.output)-1):
      self.output[i+1] = np.tanh(self.output_before[i]+(self.w[i+1]*self.output_before[i+1]))
    
    self.output_before = self.output

    return self.output
    
class MotorNeurons:
    def __init__(self, isVRN=False):
        self.isVRN = isVRN

        self.bias_tjoint = 0.0
        self.bias_cjoint = 1.2
        self.bias_fjoint = -0.6
        self.WeightM0_H2 = -0.5
        self.WeightM1_H2 = 0.5
        self.WeightM2_H2 = -0.5
        self.WeightM3_H2 = -0.5
        self.WeightM4_H2 = 0.5
        self.WeightM5_H2 = -0.5
        self.WeightM6_H1 = -0.5
        self.WeightM7_H1 = 0.5
        self.WeightM8_H1 = -0.5
        self.WeightM9_H1 = 0.5
        self.WeightM10_H1 = -0.5
        self.WeightM11_H1 = 0.5
        self.WeightM12_H1 = -0.2
        self.WeightM13_H1 = 0.2
        self.WeightM14_H1 = -0.2
        self.WeightM15_H1 = 0.2
        self.WeightM16_H1 = -0.2
        self.WeightM17_H1 = 0.2

        self.WeightVrnM0_H2 = 0.5
        self.WeightVrnM1_H2 = -0.5
        self.WeightVrnM2_H2 = 0.5

        self.WeightM3_VrnH1 = 0.5
        self.WeightM4_VrnH1 = -0.5
        self.WeightM5_VrnH1 = 0.5

        self.WeightposVrn_Motor = 0.5

    def map_input_to_motors(self, H1, H2, vrnH1=0, vrnH2=0):

        if self.isVRN:
            m0 = self.WeightVrnM0_H2 * vrnH2 + self.bias_tjoint
            m1 = self.WeightVrnM1_H2 * vrnH2 + self.bias_tjoint
            m2 = self.WeightVrnM2_H2 * vrnH2 + self.bias_tjoint

            m3 = self.WeightM3_VrnH1 * vrnH1 + self.bias_tjoint
            m4 = self.WeightM4_VrnH1 * vrnH1 + self.bias_tjoint
            m5 = self.WeightM5_VrnH1 * vrnH1 + self.bias_tjoint
        else:
            m0 = self.WeightM0_H2 * H2 + self.bias_tjoint
            m1 = self.WeightM1_H2 * H2 + self.bias_tjoint
            m2 = self.WeightM2_H2 * H2 + self.bias_tjoint

            m3 = self.WeightM3_H2 * H2 + self.bias_tjoint
            m4 = self.WeightM4_H2 * H2 + self.bias_tjoint
            m5 = self.WeightM5_H2 * H2 + self.bias_tjoint

        m6 = self.WeightM6_H1 * H1 + self.bias_cjoint
        m7 = self.WeightM7_H1 * H1 + self.bias_cjoint
        m8 = self.WeightM8_H1 * H1 + self.bias_cjoint

        m9 = self.WeightM9_H1 * H1 + self.bias_cjoint
        m10 = self.WeightM10_H1 * H1 + self.bias_cjoint
        m11 = self.WeightM11_H1 * H1 + self.bias_cjoint

        m12 = 0 + self.bias_fjoint
        m13 = 0 + self.bias_fjoint
        m14 = 0 + self.bias_fjoint

        m15 = 0 + self.bias_fjoint
        m16 = 0 + self.bias_fjoint
        m17 = 0 + self.bias_fjoint

        return [m0, m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12, m13, m14, m15, m16, m17]


class CPG:
    def __init__(self):
        self.outputH1 = 0.001
        self.outputH2 = 0.001
        self.activityH1 = 0.0
        self.activityH2 = 0.0

        self.WeightH1_H1 = 1.2
        self.WeightH2_H2 = 1.2
        self.WeightH1_H2 = -0.2
        self.WeightH2_H1 = 0.2

        self.set_frequency()
        self.outH1history = []
        self.outH2history = []

    def set_frequency(self, sigma=0.02 * np.pi):
        a = 1.01 * np.array([[np.cos(sigma), np.sin(sigma)], [-np.sin(sigma), np.cos(sigma)]])

        self.WeightH1_H1 = a[0, 0]
        self.WeightH2_H2 = a[1, 1]
        self.WeightH1_H2 = a[0, 1]
        self.WeightH2_H1 = a[1, 0]

    def update_history(self):
        if len(self.outH2history) > 100:
            self.outH2history.pop(0)
            self.outH1history.pop(0)

        self.outH1history.append(self.outputH1)
        self.outH2history.append(self.outputH2)

    def update(self):
        activityH1 = self.WeightH1_H1 * self.outputH1 + self.WeightH1_H2 * self.outputH2
        activityH2 = self.WeightH2_H2 * self.outputH2 + self.WeightH2_H1 * self.outputH1
        self.outputH1 = tanh(activityH1)
        self.outputH2 = tanh(activityH2)

        self.update_history()

        return self.outputH1, self.outputH2


class VRN:
    def __init__(self):
        self.output_final = 1.0
        self.Bias = -2.48285

        self.Weight1_input = 1.7246
        self.Weight2_input = -1.7246
        self.Weight3_input = -1.7246
        self.Weight4_input = 1.7246

        self.Weight1_cpg = 1.7246
        self.Weight2_cpg = -1.7246
        self.Weight3_cpg = 1.7246
        self.Weight4_cpg = -1.7246

        self.WeightOut_1 = 0.5
        self.WeightOut_2 = 0.5
        self.WeightOut_3 = -0.5
        self.WeightOut_4 = -0.5

        self.outHistory = []
        self.output = None

    def update_history(self):
        if len(self.outHistory) > 100:
            self.outHistory.pop(0)

        self.outHistory.append(self.output)

    def pass_input(self, input_activation, cpg_input):
        activity1 = cpg_input * self.Weight1_cpg + input_activation * self.Weight1_input + self.Bias
        output1 = tanh(activity1)

        activity2 = cpg_input * self.Weight2_cpg + input_activation * self.Weight2_input + self.Bias
        output2 = tanh(activity2)

        activity3 = cpg_input * self.Weight3_cpg + input_activation * self.Weight3_input + self.Bias
        output3 = tanh(activity3)

        activity4 = cpg_input * self.Weight4_cpg + input_activation * self.Weight4_input + self.Bias
        output4 = tanh(activity4)

        activity_out = output1 * self.WeightOut_1 + output2 * self.WeightOut_2 + output3 * self.WeightOut_3 + output4 * self.WeightOut_4

        self.output = tanh(activity_out)

        self.update_history()

        return self.output


class RBFN:
    def __init__(self, n_in, n_means, n_out):
        self.n_in = n_in
        self.n_out = n_out
        self.n_means = n_means
        self.means = np.array([[0.197502605244714, 0.0116134710983495],
                               [0.186262330054474, -0.0513215523826319],
                               [0.160108190512079, -0.112223321373292],
                               [0.121345530919906, -0.161040594190929],
                               [0.0719860651844074, -0.189923814297374],
                               [0.0141963576606884, -0.197606271984908],
                               [-0.0486434550084813, -0.187058047459262],
                               [-0.109824648748977, -0.161482000254625],
                               [-0.159350021786094, -0.123210316349248],
                               [-0.189146572072207, -0.0742568326539746],
                               [-0.197676570639880, -0.0167696789424222],
                               [-0.187826471100979, 0.0459628096348897],
                               [-0.162832761843639, 0.107404511728045],
                               [-0.125055778037444, 0.157622662467458],
                               [-0.0765121272287780, 0.188330326327298],
                               [-0.0193331257560796, 0.197713811016137],
                               [0.0432804335311038, 0.188567829847966],
                               [0.104963998070186, 0.164160616822661],
                               [0.155859219238491, 0.126881945209503],
                               [0.187475215215534, 0.0787518652690301]])

        self.beta = 25.0
        self.W = np.random.uniform(-0.0, 0.0, (n_means, n_out))

        self.plot = False
        self.histIdx = 0
        self.lengthHist = 100
        self.activationHist = np.zeros((self.lengthHist, n_means))
        self.outHist = np.zeros((self.lengthHist, n_out))

    def get_weights(self):
        return self.W.ravel()

    @staticmethod
    def scale(x, out_range=(-1, 1)):
        domain = np.min(x), np.max(x)
        y = (x - (domain[1] + domain[0]) / 2) / (domain[1] - domain[0])
        return y * (out_range[1] - out_range[0]) + (out_range[1] + out_range[0]) / 2

    @staticmethod
    def limit_weighs(weights):
        lim = 1 / 6.2347 * 0.45
        return np.clip(weights, -lim, lim)

    def set_weights(self, weights):
        w_flattened = np.array(weights)
        # w_flattened = self.limit_weighs(w_flattened)
        self.W = np.reshape(w_flattened, (self.n_means, self.n_out))

        # self.W = sklearn.preprocessing.scale( self.W, axis=0, with_mean=True, with_std=True, copy=True )
        # self.W = sklearn.preprocessing.normalize(self.W, norm="l1", axis=0)

        return self.W

    def _basisfunc(self, my, i):
        return np.exp(-self.beta * (np.linalg.norm(i - my) ** 2))

    def _calc_act(self, inputs):
        # calculate activations of RBFs
        a = np.zeros(self.n_means, float)

        for i, mean in enumerate(self.means):
            a[i] = self._basisfunc(mean, inputs)

        return a

    def _save_hist(self, activations, out):
        self.activationHist[self.histIdx] = activations
        self.outHist[self.histIdx] = out
        self.histIdx = (self.histIdx + 1) % self.lengthHist

    def forward(self, inputs):
        a = self._calc_act(inputs)
        out = np.dot(a, self.W)

        if self.plot:
            self._save_hist(a, out)

            if self.histIdx == 0:
                plt.subplot(121)
                plt.cla()
                plt.plot(self.activationHist)
                plt.subplot(122)
                plt.cla()
                plt.plot(self.outHist)
                plt.suptitle('Categorical Plotting')
                plt.pause(0.01)

        return out


class InputPreprocessor:
    def __init__(self):

        self.activity1 = 0.0
        self.activity2 = 0.0

        self.output1 = 0.01
        self.output2 = 0.01

        self.Weight1_1 = 5.4
        self.Weight2_2 = 5.4
        self.Weight1_2 = -3.55
        self.Weight2_1 = -3.55

        self.WeightIRL = 7
        self.WeightIRR = 7

        self.input1 = 1
        self.input2 = 1

    @staticmethod
    def map_variable(value, inMin, inMax, outMin, outMax):
        in_range = inMax - inMin
        out_range = outMax - outMin
        value_mapped = float(value - inMin) / float(in_range)

        return outMin + (value_mapped * out_range)

    def set_input(self, I1, I2):

        self.input1 = self.map_variable(I1, 0.00, 0.55, -1, 1)
        self.input2 = self.map_variable(I2, 0.00, 0.55, -1, 1)

        if I1 == -1:
            self.input1 = 1
        if I2 == -1:
            self.input2 = 1

    def update(self):
        self.activity1 = self.input2 * self.WeightIRR + self.output1 * self.Weight1_1 + self.output2 * self.Weight1_2
        self.output1 = tanh(self.activity1)

        self.activity2 = self.input1 * self.WeightIRL + self.output2 * self.Weight2_2 + self.output1 * self.Weight2_1
        self.output2 = tanh(self.activity2)

        return self.output1, self.output2
