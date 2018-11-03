# Inspired by https://nptel.ac.in/courses/Webcourse-contents/IIT%20Kharagpur/Structural%20Analysis/pdf/m4l25.pdf

from personalMathLib import Matrix, Point
import math as mh

# Global Young's modulos
E = 200000


class HSS:

    def __init__(self, inputLine):
        vals = inputLine.split(' ')
        self.designation = vals[0]
        self.size = vals[1]
        self.mass = float(vals[2])
        self.deadLoad = float(vals[3])
        self.area = float(vals[4])
        self.inertia = float(vals[5])
        self.s = float(vals[6])
        self.slenderness = float(vals[7])
        self.dims = []
        for v in self.designation.split('x'):
            self.dims = self.dims + [float(v)] # dims[0] == dims[1]
        # Add more data when added to txt

    def getDes(self):
        return self.designation

    def checkAcceptable(self, area, intertia):
        # Check area and intertia
        pass


class Member:

    def __init__(self, targetA, targetB, designation):
        # Targets are designation strings for points
        self.a = str(targetA)
        self.b = str(targetB)
        self.id = designation
        self.hasJoints = False
        self.answerForm = False
        self.structureIteration = 0

    def setJoints(self, joints):
        self.jointA = joints[0]
        self.jointB = joints[1]
        if self.jointA.y > self.jointB.y:
            temp = self.jointA
            self.jointA = self.jointB
            self.jointB = temp
        self.hasJoints = True
        self.isHorizontal = True if (self.jointA.y - self.jointB.y == 0) else False
        self.height = None if not self.isHorizontal else self.jointA.y
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
        self.lengthChange = (self.length*1000*self.force)/(E*self.structure.area)
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
        if self.hasJoints:  # Set both as they set together
            newM.jointA = self.jointA.duplicate()
            newM.jointB = self.jointB.duplicate()
        return newM

    def pickHSS(self, HSSs, minForce=None, maxLength=None):
        if self.answerForm is False:
            self.force = self.force * (-1)
            self.answerForm = True
        # HSSs is a list of all HSS objects
        crushStress = 350
        tensileStress = 350
        lowestI = 0
        lowestA = 0
        if minForce is None:
            minForce = self.force
        if maxLength is None:
            maxLength = self.length
        if minForce < 0:
            lowestA = abs((2*minForce)/crushStress)
            lowestI = abs((3*minForce*maxLength*maxLength)/(mh.pi*mh.pi*E))
        else:
            lowestA = (2*minForce)/tensileStress
        slenderness = maxLength*10/2
        # find HSS
        possible = []
        for h in HSSs:
            if h.area >= lowestA and h.inertia >= lowestI:
                if minForce >= 0:
                    if h.slenderness < slenderness:
                        continue
                possible = possible + [h]
        it = 0
        order = []
        while len(order) <= self.structureIteration:
            currentLowest = None
            reduced = []
            for p in possible:
                if currentLowest is None or p.dims[0] < currentLowest:
                    currentLowest = p.dims[0]
                    reduced = reduced[len(reduced)-(1+it):len(reduced)-1] + [p]
                elif p.dims[0] == currentLowest:
                    reduced = reduced + [p]
            order = []
            finalLow = None
            #print(reduced)
            for p in reduced:
                if finalLow is None or p.dims[2] < finalLow:
                    finalLow = p.dims[2]
                    order = order + [p]
            it = it + 1
        # Incorporate the structure of the member
        #print(order)
        #print(len(order)-(1+self.structureIteration))
        self.structure = order[len(order) - (1+self.structureIteration)]
        self.structureIteration = self.structureIteration + 1
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
        self.forcesCalced = False

    def fetchJoint(self, desi):
        des = str(int(round(float(desi))))
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
        # dispMatrix.display()
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
            if d.disp is None:
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
        # result.display()
        # print(self.DOFs[0].disp)
        # Assign displacements to DOFs
        for d in self.DOFs:
            for m in result.getLabels('row'):
                if d.id == m:
                    d.setDisp(result.getValuebyLabels(d.id, 1))
        return True

    def display(self):
        for m in self.members:
            print(m.id, "-->", slideRuleAccuracy(m.force))
        return True

    def getAnswerForces(self):
        # Convert from member forces on joins to joint forces on members
        self.forcesCalced = True
        if not self.forcesCalced:
            for m in self.members:
                if m.answerForm is False:
                    m.force = m.force * -1
                    m.answerForm = True
        return True

    def selectSpan(self, height):
        membersList = []
        hori = []
        heights = []
        ext = None
        for m in self.members:
            if m.isHorizontal:
                hori = hori + [m]
                heights = heights + [m.height]
        if height == 'lower':
            for h in heights:
                if ext is None:
                    ext = h
                else:
                    if ext > h:
                        ext = h
        elif height == 'height':
            for h in heights:
                if ext is None:
                    ext = h
                else:
                    if ext < h:
                        ext = h
        for l in self.members:
            if l.height == ext:
                membersList = membersList + [l.id]
        return membersList

    def chooseHSSs(self, HSSList, bottomChordIds, topChordIds):
        topForce = 0
        topLength = 0
        botForce = 0
        botLength = 0
        webForce = 0
        webLength = 0
        for m in self.members:
            placed = False
            for id in bottomChordIds:
                if m.id == id:
                    if botForce < abs(m.force):
                        botForce = abs(m.force)
                    if botLength < m.length:
                        botLength = m.length
                    placed = True
                    break
            if placed:
                continue
            for id in topChordIds:
                if m.id == id:
                    if topForce < abs(m.force):
                        topForce = abs(m.force)
                    if topLength < m.length:
                        topLength = m.length
                    placed = True
                    break
            if placed:
                continue
            if webForce < abs(m.force):
                webForce = abs(m.force)
            if webLength < m.length:
                webLength = m.length
        # Get force signs
        #if
        # Set top, bottom and web HSSs
        for m in self.members:
            set = False
            for id in bottomChordIds:
                if m.id == id:
                    m.pickHSS(HSSList, botForce, botLength)
                    set = True
                    break
            if set:
                continue
            for id in topChordIds:
                if m.id == id:
                    m.pickHSS(HSSList, topForce, topLength)
                    set = True
                    break
            if set:
                continue
            # Selects HSS for the web forces
            m.pickHSS(HSSList, -webForce, webLength)
        for m in self.members:
            m.calculateLengthChange()  # this is appropriate as it is calculated based on HSS
        return True

    def getMemberByID(self, id):
        for m in self.members:
            if m.id == id:
                return m.duplicate()

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

class Variable:

    def __init__(self, varName, value):
        self.name = varName
        self.val = float(value)


def getSlideValue(strVal, leng):
    type = int
    final = ''
    x = 0
    followsPoint = False
    # Check if there is a decimal in the range
    for i in range(0, leng, 1):
        if i >= len(strVal):
            break # Exit early if done
        if strVal[i] == '.':
            leng = leng + 1
            break
    for i in range(0, leng, 1):
        if followsPoint:
            followsPoint = False
            continue
        if i >= len(strVal):
            return [final, type] # Exit early if done
        if strVal[i] == '.':
            final = final + "." + strVal[i + 1]
            followsPoint = True
            x = 1
            type = float
        else:
            final = final + strVal[i]
    i = i + 1
    if i + 1 >= len(strVal):
        return [final, type] # Exit early if done now
    # Check for rounding
    # Get rounded value
    z = 1
    while True:
        print(final)
        roundVal = int(final[len(final)-z]) + 1
        if (len(str(roundVal)) > z):
            z = z + 1
        else:
            roundVal = roundVal * mh.pow(10, z)
            break
    roundVal = int(roundVal)
    if strVal[i] != '.' and int(strVal[i]) > 4:
        final = final[0:len(final)-z] + str(roundVal)
    elif strVal[i] == '.' and int(strVal[i+1]) > 4:
        roundVal = int(roundVal / 10)
        final = final[0:len(final)-z] + str(roundVal)
    return [final, type]

def slideRuleAccuracy(val):
    type = int
    strVal = str(val)
    is0Dec = False
    neg = False
    if (val < 0):
        strVal = strVal[1:len(strVal)]
        neg = True
    if (val < 1) and (val > -1):
        type = float
        strVal = strVal[2:len(strVal)] # remove the 0.
        is0Dec = True
    # Remove leading 0s
    zeros = ''
    for v in strVal:
        if v == '0':
            zeros = zeros + '0'
        else:
            break
    strVal = strVal[len(zeros):len(strVal)]
    # Use slide rule on sig digs
    if (strVal[0] == '1'):
        res = getSlideValue(strVal, 4)
    else:
        res = getSlideValue(strVal, 3)
    final = res[0]
    if res[1] == float:
        type = float
    # Add values back
    if is0Dec:
        final = "0." + zeros + final
    # Cast to Type
    if type == int:
        result = int(final)
    else:
        result = float(final)
    if neg:
        return -result
    else:
        return result

def interpVar(value, variables):
    val = None
    try:
        val = float(value)
        if val == int(value):
            val = int(value)
    except:
        for var in variables:
            if value == var.name:
                val = var.val
                break
        if val is None:
            return value # A string
    return val

def subtract(oldStr, sub):
    if sub == 'xy' or oldStr == '':
        return ''
    elif sub == 'x':
        if oldStr == 'x':
            return ''
        else:
            return 'y'
    elif sub == 'y':
        if oldStr == 'y':
            return ''
        else:
            return 'x'
    else:
        return 'xy'

def test():
    print(slideRuleAccuracy(-300.91368421052357))


#test()
