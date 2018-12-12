import math as mh
import civLib as civ

E = 4000
u = 0.2
stress_t = 30  # Tension
stress_c = 6  # Compression
stress_sm = 4  # Sheer Matboard
stress_sg = 2  # Sheer Glue


class Sheer_Plane:
    def __init__(self, Q, b):
        self.Q = Q
        self.b = b


class Safety:
    def __init__(self):
        self.comp = 0
        self.tens = 0
        self.glueSheer = 0
        self.matSheer = 0

    def setComp(self, val):
        self.comp = val
        return True

    def setTen(self, val):
        self.tens = val
        return True

    def setGSheer(self, val):
        self.glueSheer = val
        return True

    def setMSheer(self, val):
        self.matSheer = val
        return True

    def display(self):
        print("---FACTORS OF SAFETY---")
        print("Flexural")
        print("---------")
        print("Compressive ->", civ.slideRuleAccuracy(self.comp))
        print("Tensile ->", civ.slideRuleAccuracy(self.tens))
        print("Sheers")
        print("---------")
        print("Glue ->", civ.slideRuleAccuracy(self.glueSheer))
        print("Matboard ->", civ.slideRuleAccuracy(self.matSheer))
        return True


class Max:
    def __init__(self):
        # Set defaults
        self.comp = 0
        self.tens = 0
        self.glueSheer = 0
        self.matSheer = 0
        self.plateCases = [0, 0, 0, 0]

    def setComp(self, val):
        self.comp = val
        return True

    def setTen(self, val):
        self.tens = val
        return True

    def setGSheer(self, val):
        self.glueSheer = val
        return True

    def setMSheer(self, val):
        self.matSheer = val
        return True

    def setPlates(self, valList):
        self.plateCases = valList
        return True

    def display(self):
        print("---MAXIMUM---")
        print("Flexural")
        print("---------")
        print("Compressive ->", civ.slideRuleAccuracy(self.comp))
        print("Tensile ->", civ.slideRuleAccuracy(self.tens))
        print("Sheers")
        print("---------")
        print("Glue ->", civ.slideRuleAccuracy(self.glueSheer))
        print("Matboard ->", civ.slideRuleAccuracy(self.matSheer))
        print("Plate Buckling")
        print("---------")
        print("Case 1 ->", civ.slideRuleAccuracy(self.plateCases[0]))
        print("Case 2 ->", civ.slideRuleAccuracy(self.plateCases[1]))
        print("Case 3 ->", civ.slideRuleAccuracy(self.plateCases[2]))
        print("Case 4 ->", civ.slideRuleAccuracy(self.plateCases[3]))
        return True


class Section:
    def __init__(self, I, yBar, h, a, b, t, height, planes, name="Section"):
        self.I = I
        self.yBar = yBar
        self.a = a  # Greatest Distance between diaphragms
        self.b = b  # Bases array for plates (case order)
        self.t = t  # Thickness array for plates (case order)
        self.h = h
        self.height = height
        self.SpM = planes[0]
        self.SpG = planes[1]
        self.name = name
        self.y = abs(self.height - self.yBar)
        self.max = Max()
        self.safety = Safety()
        self.Mt = 0
        self.Mb = 0
        self.Vt = 0
        self.Vb = 0

    def getFlexural(self, M):
        stresses = [stress_c, stress_t]
        res = []
        f = True
        for s in stresses:
            sp = self.yBar if not f else self.y
            res = res + [(self.I*s)/(M*sp)]
            f = False
        self.max.setComp(res[0])
        self.max.setTen(res[1])
        self.Mb = M
        return True

    def getSheer(self, V):
        planes = [self.SpM, self.SpG]
        sheers = [stress_sm, stress_sg]
        res = []
        for c in range(0, 2, 1):
            res = res + [(sheers[c]*self.I*planes[c].b)/(V*planes[c].Q)]
        self.max.setMSheer(res[0])
        self.max.setGSheer(res[1])
        self.Vb = V
        return True

    def getBuckling(self):
        coeffs = [4, 0.425, 6, 5]
        p = []
        round = 0
        if self.Mb == 0:
            raise ValueError("Flexural calculations should be completed first")
            return False
        for val in coeffs:
            k = (self.y*self.Mb)/self.I
            if round != 3:
                multiplier = (self.t[int(round/2)]/self.b[round])**2
            else:
                multiplier = (self.t[int(round/2)]/self.h)**2 + (self.t[int(round/2)]/self.a)**2
            stress = (val*(mh.pi**2)*E)/(12*(1-(u**2)))*multiplier
            p = p + [stress/k]
            round = round + 1
        self.max.setPlates(p)
        return True

    def getFOSs(self, M, V):
        self.Mt = M
        self.Vt = V
        self.safety.setComp(stress_c/((self.Mt*self.y)/self.I))
        self.safety.setTen(stress_t/((self.Mt*self.yBar)/self.I))
        sheers = [stress_sm, stress_sg]
        planes = [self.SpM, self.SpG]
        res = []
        for c in range(0, 2, 1):
            res = res + [sheers[c]/((self.Vt*planes[c].Q)/(self.I*planes[c].b))]
            print(((self.Vt*planes[c].Q)/(self.I*planes[c].b)))
        self.safety.setMSheer(res[0])
        self.safety.setGSheer(res[1])
        return True

    def display(self):
        print("=-=-=-=-=-=-=-=-=")
        print(self.name)
        self.max.display()
        self.safety.display()
        return True

    def setName(self, name):
        self.name = name
        return True

    def duplicate(self):
        other = Section(self.I, self.yBar, self.h, self.a, self.b, self.t, self.height, [self.SpM, self.SpG], self.name)
        return other


def main():
    Mt = 54797
    Vt = 66.7
    Mb = 166.1
    Vb = 0.698
    I = 1261000
    t = [2*1.27, 1.27]  # Cases 1-2, 3-4
    b = [80, 30, 135/2]  # Cases 1,2,3
    h = 135
    a = 190
    y = 99.0
    height = 137.54
    name = "Section A"
    planes = [Sheer_Plane(2*99.0*1.27*(1/2)*99.0, 2*1.27), Sheer_Plane(12*100*1.27*38.57, 30)]  # Matboard then glue
    section3 = Section(I, y, h, a, b, t, height, planes, name)
    section3.getFlexural(Mb)
    section3.getSheer(Vb)
    section3.getBuckling()
    section3.getFOSs(Mt, Vt)
    section3.display()
    return True

main()

def test():
    print(4*mh.sqrt(2)/3 - 2/3)

test()

# p = []
# M = 95.13
# I = 1034000
# E = 4000
# u = 0.2
# t = [1.27, 1.27]
# b = [80, 30, 135/2]
# h = 135
# a = 130
# y = 135 + t - 89.56
# [803.8738320583466, 607.3713397774172, 1693.7588560241707, 733.3984198452785]
