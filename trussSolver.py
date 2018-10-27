from personalMathLib import Matrix, Point
import math as mh


class Member:

    def __init__(self, targetA, targetB):
        # Targets are designation strings for points
        self.a = str(targetA)
        self.b = str(targetB)

    def setJoints(self, joints):
        self.jointA = joints[0]
        self.jointB = joints[1]
        if self.jointA.y < self.jointB.y:
            temp = self.jointA
            self.jointA = self.jointB
            self.jointB = temp

    def getJointTargets(self):
        return [self.a, self.b]

    def calculate(self):
        self.length = self.jointA.distance(self.jointB)
        self.sine = (self.jointB.y - self.jointA.y)/self.length
        self.cosine = (self.jointB.x - self.jointA.x)/self.length
        # MAKE MATRIX
        # Define 4 vals
        # DOFS are [a,a,b,b]
        r1 = Matrix([[self.cosine*self.cosine, self.sine*self.cosine]])
        r2 = Matrix([[self.sine*self.cosine, self.sine*self.sine]])
        # make pattern
        mTempList = [r1.getRow[0] + r1.returnScale(-1).getRow[0]]
        mTempList += [r2.getRow[0] + r2.returnScale(-1).getRow[0]]
        mTempList += [r1.returnScale(-1).getRow[0] + r1.getRow[0]]
        mTempList += [r2.returnScale(-1).getRow[0] + r2.getRow[0]]
        return True


class Truss:

    def __init__(self, jointList, memberList):
        self.joints = jointList
        self.members = memberList

        for m in self.members:
            js = []
            for j in m.getJointTargets:
                js = js + [self.fetchJoint(j)]
            for j in js:
                if j == False:
                    raise ValueError("Yo this doesn't work")
            m.setJoints(js)  # Sets the joints into the Member
            m.calculate()

    def fetchJoint(self, des):
        for j in self.joints:
            if j.getName() == des:
                return j
        return False


class Joint(Point):

    def __init__(self, point, DOFs):
        self.x = point.x
        self.y = point.y
        self.designation = point.designation
        self.DOFids = [DOFs[0], DOFs[1]]


def main():
    points = [Point(0, 0), Point(2.5, mh.sqrt(18.75)), Point(5, 0), Point(7.5, mh.sqrt(18.75)), Point(10, 0)]
    # Set designations for undesignated points
    i = 0
    for p in points:
        i = i + 1
        if p.getName() == "":
            p.setName(str(i))
            # Points use number notations in order unless otherwise specified
    joints = []
    i = 1
    for p in points:
        joints = joints + Joint(p, [i*2-1, i*2])
    members = [Member(1, 2), Member(1, 3), Member(2, 4), Member(3, 5), Member(4, 5)]
    truss = Truss(joints, members)

    return True

main()
