# Inspired by https://nptel.ac.in/courses/Webcourse-contents/IIT%20Kharagpur/Structural%20Analysis/pdf/m4l25.pdf

from personalMathLib import Matrix, Point
import math as mh

# Global Young's modulos
E = 200000

class Member:

    def __init__(self, targetA, targetB, designation):
        # Targets are designation strings for points
        self.a = str(targetA)
        self.b = str(targetB)
        self.id = designation
        self.hasJoints = False

    def setJoints(self, joints):
        self.jointA = joints[0]
        self.jointB = joints[1]
        if self.jointA.y > self.jointB.y:
            temp = self.jointA
            self.jointA = self.jointB
            self.jointB = temp
        self.hasJoints = True
        return True

    def buildDisp(self, joints):
        self.setJoints(joints)  # Reset joints to include proper forces
        self.dispMatrix = Matrix.setDims(4, 1)
        self.dispMatrix.setRowLabels(self.getDOFs())
        for dof in self.getDOFData():
            self.dispMatrix.setValueByLabels(dof.id, '', dof.disp)
        return True

    def getDOFData(self):
        dofs = []
        for dof in self.jointA.DOFs:
            dofs = dofs + [dof.duplicate()]
        for dof in self.jointB.DOFs:
            dofs = dofs + [dof.duplicate()]
        return dofs

    def calculateLengthChange(self):
        # trueA = self.jointA.applyDisp()
        # trueB = self.jointB.applyDisp()
        # newLength = trueA.distance(trueB)
        # self.lengthChange = newLength - self.length
        self.lengthChange = (self.length*1000*self.force)/(E*self.area)
        return True

    def getForce(self):
        stiffnessRepresentation = Matrix([[self.cosine, self.sine, -self.cosine, -self.sine]])
        stiffnessRepresentation.scale(1/self.length)
        # print("-->", self.id, "<--")
        # self.dispMatrix.display()
        # print("---")
        # stiffnessRepresentation.display()
        result = stiffnessRepresentation * self.dispMatrix
        self.force = result.getValue(0, 0)
        return True

    def getDOFs(self):
        dofs = []
        for dof in self.jointA.DOFs:
            dofs = dofs + [dof.id]
        for dof in self.jointB.DOFs:
            dofs = dofs + [dof.id]
        return dofs

    def getJointTargets(self):
        return [self.a, self.b]

    def calculate(self):
        self.length = self.jointA.distance(self.jointB)
        self.sine = (self.jointB.y - self.jointA.y)/self.length
        self.cosine = (self.jointB.x - self.jointA.x)/self.length
        # DOFS are [a,a,b,b]
        # Define 4 vals
        dofs = self.getDOFs()
        r1 = Matrix([[self.cosine*self.cosine, self.sine*self.cosine]])
        r2 = Matrix([[self.sine*self.cosine, self.sine*self.sine]])
        # make pattern
        mTempList = [r1.getRow(0) + r1.returnScale(-1).getRow(0)]
        mTempList += [r2.getRow(0) + r2.returnScale(-1).getRow(0)]
        mTempList += [r1.returnScale(-1).getRow(0) + r1.getRow(0)]
        mTempList += [r2.returnScale(-1).getRow(0) + r2.getRow(0)]
        self.stiffnessMatrix = Matrix(mTempList, dofs, dofs)
        self.stiffnessMatrix.scale(1/self.length)
        return True

    def duplicate(self):
        newM = Member(self.a, self.b, self.id)
        if self.hasJoints: # Set both as they set together
            newM.jointA = self.jointA.duplicate()
            newM.jointB = self.jointB.duplicate()
        return newM

    def pickHSS(self):
        # Get from HSS library (json?) with params
        self.HSSString = "123x123x123"
        self.area = 4840
        return True


def checkUnique(list, value):
    for val in list:
        if value == val:
            return False
    return True


class Truss:

    def __init__(self, jointList, memberList):
        self.joints = []
        self.members = []
        for j in jointList:
            self.joints = self.joints + [j.duplicate()]
        for m in memberList:
            self.members = self.members + [m.duplicate()]
        for m in self.members:
            js = []
            for j in m.getJointTargets():
                js = js + [self.fetchJoint(j)]
            for j in js:
                if not j:
                    raise ValueError("Yo this doesn't work")
            m.setJoints(js)  # Sets the joints into the Member
            m.calculate()  # Calculates the relevant dimensions for member matricies
            # m.stiffnessMatrix.display()
        self.compileStiffnesses()

    def fetchJoint(self, desi):
        des = str(desi)
        for j in self.joints:
            if j.getName() == des:
                return j
        return False

    def compileDofs(self):
        self.DOFs = []
        for j in self.joints:
            for d in j.DOFs:
                self.DOFs = self.DOFs + [d]
        return True

    def compileStiffnesses(self):
        uniqueDofs = []
        for ma in self.members:
            for dof in ma.stiffnessMatrix.getLabels('row'):
                if checkUnique(uniqueDofs, dof):
                    uniqueDofs = uniqueDofs + [dof]
        # sort the dofs
        # Make new blank matrix
        self.stiffness = Matrix.setDims(len(uniqueDofs), len(uniqueDofs))
        self.stiffness.setRowLabels(uniqueDofs)
        self.stiffness.setColLabels(uniqueDofs)
        # Sum by dof
        for r in range(0, self.stiffness.rows, 1):
            for c in range(0, self.stiffness.cols, 1):
                selectedLabels = [self.stiffness.getRowLabel(r), self.stiffness.getColLabel(c)]
                value = 0
                for m in self.members:
                    mDis = m.stiffnessMatrix
                    print("---")
                    mDis.display()
                    if not mDis.getValuebyLabels(selectedLabels[0], selectedLabels[1]) == False:
                        value = value + mDis.getValuebyLabels(selectedLabels[0], selectedLabels[1])
                self.stiffness.setValue(r, c, value, True)
                # print(self.stiffness.M[r][c])
        # self.stiffness.display()
        return True

    def calculateJointForces(self):
        # Build matrix with dofs of unknown forces
        unknownDofs = []
        knownDisps = []
        for d in self.DOFs:
            # print(d.disp)
            # print(d.force)
            if d.force is None:
                unknownDofs = unknownDofs + [d]
            else:
                knownDisps = knownDisps + [d]
        rows = []
        # Build known displacements matrix
        disps = []
        dispLabels = []
        for u in knownDisps:
            disps = disps + [u.disp]
            dispLabels = dispLabels + [u.id]
        dispMatrix = Matrix(disps, dispLabels)
        #dispMatrix.display()
        # Get mini stiffness matrix
        for u in unknownDofs:
            rows = rows + [u.id]
        miniM = self.stiffness.getSubByLabels(rows, dispMatrix.rowLabels)
        # miniM.display()
        # print("----")
        # print("s")
        # miniM.display()
        # dispMatrix.display()
        # print("s")
        # Calculate forces
        forces = miniM * dispMatrix
        forces.setRowLabels(rows)
        forces.setColLabels([1])
        # Assign forces to DOFs
        for d in self.DOFs:
            for m in forces.getLabels('row'):
                if d.id == m:
                    d.setForce(forces.getValuebyLabels(d.id, 1))
        return True

    def calculateMemberForces(self):
        for m in self.members:
            js = []
            for j in m.getJointTargets():
                js = js + [self.fetchJoint(j)]
            m.buildDisp(js)
            m.getForce()
        return True

    def calculateDisplacements(self):
        self.compileDofs()
        # Build matrix with dofs of unknown displacement
        unknownDofs = []
        for d in self.DOFs:
            # print(d.force)
            # print(d.disp)
            if d.disp == None:
                unknownDofs = unknownDofs + [d]
        rows = []
        forces = []
        for u in unknownDofs:
            rows = rows + [u.id]
            forces = forces + [[u.force]]
        forcesMatrix = Matrix(forces, rows, rows)
        miniM = self.stiffness.getSubByLabels(rows, rows)
        miniM.invert()
        # Solved joints
        result = miniM * forcesMatrix
        result.setColLabels([1])
        result.setRowLabels(miniM.colLabels)
        #result.display()
        # print(self.DOFs[0].disp)
        # Assign displacements to DOFs
        for d in self.DOFs:
            for m in result.getLabels('row'):
                if d.id == m:
                    d.setDisp(result.getValuebyLabels(d.id, 1))
        return True

    def display(self):
        for m in self.members:
            print(m.id, "-->", m.force)
        return True

    def getAnswerForces(self):
        # Convert from member forces on joins to joint forces on members
        for m in self.members:
            m.force = m.force * -1
        return True

    def chooseHSSs(self):
        for m in self.members:
            m.pickHSS()
            m.calculateLengthChange()  # this is appropriate as it is calculated based on HSS
        return True

    def getDeltaR(self, virtual):
        delta = 0
        for i in range(0, len(self.members), 1):
            delta = delta + (self.members[i].lengthChange * virtual.members[i].force)
        return delta


class Joint(Point):

    def __init__(self, point, DOFids):
        super(Joint, self).__init__(point.x, point.y, point.designation)
        self.DOFs = [DOF(DOFids[0]), DOF(DOFids[1])]  # 0 is x, 1 is y

    def setForce(self, dof, val):
        if dof == 'x':
            self.DOFs[0].setForce(val)
        elif dof == 'y':
            self.DOFs[1].setForce(val)
        elif dof == 'xy':
            self.DOFs[0].setForce(val)
            self.DOFs[1].setForce(val)
        else:
            return False
        return True

    def setDisp(self, dof, val):
        if dof == 'x':
            self.DOFs[0].setDisp(val)
        elif dof == 'y':
            self.DOFs[1].setDisp(val)
        elif dof == 'xy':
            self.DOFs[0].setDisp(val)
            self.DOFs[1].setDisp(val)
        else:
            return False
        return True

    def getDOF(self, id):
        for d in self.DOFs:
            if d.id == id:
                return d
        return False

    def duplicate(self):
        dof = []
        for d in self.DOFs:
            dof = dof + [d.duplicate()]
        newJ = Joint(Point(self.x, self.y, self.designation), dof)
        return newJ

    def applyDisp(self):
        newJ = self.duplicate()
        newJ.x = self.x + self.DOFs[0].disp
        newJ.y = self.y + self.DOFs[1].disp
        return newJ


class DOF:

    def __init__(self, id):
        self.id = id
        # Nones indicate variable
        self.force = None
        self.disp = None

    def setForce(self, val):
        self.force = val
        return True

    def setDisp(self, val):
        self.disp = val
        return True

    def duplicate(self):
        dup = DOF(self.id)
        dup.force = self.force
        dup.disp = self.disp
        return dup


def main():
    height = mh.sqrt(18.75)
    points = [Point(0,0), Point(2.5, height), Point(5,0), Point(7.5, height), Point(10,0)]
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
        joints = joints + [Joint(p, [i*2-1, i*2])]
        i = i + 1
    members = [Member(1,2,1), Member(1,3,2), Member(2,3,3), Member(2,4,4), Member(3,4,5), Member(3,5,6), Member(4,5,7)]
    restrict = [[1, 'xy'], [5, 'y']]  # Disp is set 0
    force = [[2, 'xy'], [3, 'x'], [3, 'y', -20], [4, 'xy'], [5, 'x']]  # Forces are set (Default val is 0 when not specified)
    pointOfDeflection = 3
    deflectionSpan = [1,5] # Members from and to
    truss = Truss(joints, members)
    for r in restrict:
        truss.fetchJoint(r[0]).setDisp(r[1], 0)
    for f in force:
        val = 0 if len(f) == 2 else f[2]
        truss.fetchJoint(f[0]).setForce(f[1], val)
    truss.calculateDisplacements()
    truss.calculateJointForces()
    truss.calculateMemberForces()
    #print(truss.joints[0].getDOF(2).force)
    truss.getAnswerForces()
    truss.display()
    # Build list of HSSs
    #HSSFile = open('fixedData\\HSSData.txt', 'r')
    # HSSData = HSSFile.readlines()
    # HSSs = []
    # for line in HSSData:
    #     print(line)
    #     HSSs = HSSs + [line]
    truss.chooseHSSs()
    # Calculate Virtual Work
    virtualTruss = Truss(joints, members)
    for r in restrict:
        virtualTruss.fetchJoint(r[0]).setDisp(r[1], 0)
    for f in force:
        val = 0 if len(f) == 2 else f[2]
        if val != 0:
            if f[0] != pointOfDeflection:
                val = 0
            else:
                val = 1
        virtualTruss.fetchJoint(f[0]).setForce(f[1], val)
    # for j in virtualTruss.joints:
    #     print(j.DOFs[0].disp)
    virtualTruss.calculateDisplacements()
    virtualTruss.calculateJointForces()
    virtualTruss.calculateMemberForces()
    #print(truss.joints[0].getDOF(2).force)
    virtualTruss.getAnswerForces()
    #virtualTruss.display()
    print(abs(truss.getDeltaR(virtualTruss))/(1000*(points[deflectionSpan[0]-1].distance(points[deflectionSpan[1]-1]))))
    return True

main()
