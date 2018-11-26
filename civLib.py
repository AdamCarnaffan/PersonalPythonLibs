# Inspired by https://nptel.ac.in/courses/Webcourse-contents/IIT%20Kharagpur/Structural%20Analysis/pdf/m4l25.pdf

from personalMathLib import Matrix, Point
import math as mh
import sys
import time

def getTime(s):
    print("--- %s seconds ---" % (time.time() - s))

ssss = time.time()

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
        self.force = 0

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

    def getID(self):
        return self.id

    def getIteration(self):
        return self.structureIteration

    def dummy():
        # Create dummy value
        dum = Member('', '', 'Power')
        dum.jointA = Joint.dummy()
        dum.jointB = Joint.dummy()
        dum.calculate()
        dum.buildDisp([dum.jointA, dum.jointB])
        dum.getForce()
        return dum

    def calculateLengthChange(self):
        # trueA = self.jointA.applyDisp()
        # trueB = self.jointB.applyDisp()
        # newLength = trueA.distance(trueB)
        # self.lengthChange = newLength - self.length
        self.lengthChange = (self.length*1000*self.force)/(E*self.structure.area)
        return True

    def getForce(self):
        # print("ID IS",self.id)
        # print("-----")
        # print(self.sine)
        # print(self.cosine)
        stiffnessRepresentation = Matrix([[self.cosine, self.sine, -self.cosine, -self.sine]])
        if self.length != 0:
            stiffnessRepresentation.scale(1/self.length)
        # stiffnessRepresentation.display()
        # self.dispMatrix.display()
        # print("-----")
        # print("-->", self.id, "<--")
        # self.dispMatrix.display()
        # print("---")
        # stiffnessRepresentation.display()
        result = stiffnessRepresentation * self.dispMatrix
        # result.display()
        # print("*****")
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
        if self.length == 0:
            self.sine = 0
            self.cosine = 0
            return True
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
            newM.hasJoints = True
            newM.calculate()
        try:
            newM.dispMatrix = self.dispMatrix.duplicate()
            newM.stiffnessMatrix = self.stiffnessMatrix.duplicate()
        except:
            pass
        try:
            newM.force = self.force
            newM.length = self.length
        except:
            pass
        try:
            newM.height = self.height
            newM.isHorizontal = self.isHorizontal
        except:
            pass
        try:
            newM.structure = self.structure
            newM.structureIteration = self.structureIteration
        except:
            pass
        return newM

    def pickHSS(self, HSSs, minForce=None, maxLength=None, incr=False):
        if self.answerForm is False:
            self.force = self.force * (-1)
            self.answerForm = True
        # HSSs is a list of all HSS objects
        crushStress = 350
        tensileStress = 350
        lowestI = 0
        lowestA = 0
        if minForce is None:
            minForce = self.duplicate()
        if maxLength is None:
            maxLength = self.duplicate()
        if minForce.force < 0:
            lowestA = abs((2*minForce.force*1000)/crushStress)
            lowestI = abs((3*minForce.force*1000*maxLength.length*1000*maxLength.length*1000)/(mh.pi*mh.pi*E))
        else:
            lowestA = (2*minForce.force*1000)/tensileStress
        slenderness = maxLength.length*10/2
        # print(lowestI)
        # find HSS
        possible = []
        for h in HSSs:
            if h.area >= lowestA and h.inertia >= lowestI:
                if minForce.force >= 0:
                    if h.slenderness < slenderness:
                        # print(h.size)
                        continue # Skips members that don't fit the slenderness
                possible = possible + [h]
        it = 0
        if (len(possible) < 1) or (len(possible) - self.structureIteration < 1):
            self.pickCustomHSS([lowestA, lowestI, minForce.force, slenderness])
            return True
        reduced = []
        # Incorporate the structure of the member
        def HSSSort(x):
            return -x.mass
        reduced = sorted(possible, key = HSSSort)
        self.structure = reduced[len(reduced) - (1+self.structureIteration)]
        self.structure.isCustom = False
        return True


    def pickCustomHSS(self, benchmarks):
        lowestA = benchmarks[0]
        lowestI = benchmarks[1]
        minForce = benchmarks[2]
        slenderness = benchmarks[3]
        # Square therefore one dim var
        thick = 6.35
        outer = 350
        iter = 0;
        while True:
            outer = outer + thick
            inner = outer - 2*thick
            area = outer*outer - inner*inner
            inert = ((outer*outer*outer*outer) - (inner*inner*inner*inner))/12
            if area >= lowestA and inert >= lowestI:
                slend = mh.sqrt(inert/area)
                if minForce >= 0:
                    if slend > slenderness:
                        if (iter < self.structureIteration):
                            iter = iter + 1
                            continue
                        else:    
                            break
                else:
                    break
        # Generate HSS vals
        designation = str(int(outer)) + "x" + str(int(outer)) + "x" + str(int(thick))
        size = str(round(outer, 2)) + "x" + str(round(outer, 2)) + "x" + str(thick)
        w = 77*(area/(1000*1000))  # All steel
        m = (w*1000)/9.81
        customVals = designation + " " + size + " " + str(m) + " " + str(w) + " " + str(area) + " " + str(inert) + " " + "0" + " " + str(slend)
        self.structure = HSS(customVals)
        # self.structureIteration = self.structureIteration + 1
        self.structure.isCustom = True
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
            disps = disps + [[u.disp]]
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
        # miniM.display()
        # dispMatrix.display()
        # sys.exit()
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
            #print(m.force)
        return True

    def connectHSSIterations(self, previousState):
        for val in previousState:
            for m in self.members:
                if m.id == val[0]:
                    m.structureIteration = val[1]
        return True

    def getMembers(self):
        mems = []
        for m in self.members:
            mems = mems + [m.duplicate()]
        return mems

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
        # print(forces)
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
            stru = "Unset"
            try:
                stru = m.structure.size
            except:
                pass
            print(m.id, "-->", slideRuleAccuracy(m.force), "(" + stru + ")")
        return True

    def getAnswerForces(self):
        # Convert from member forces on joins to joint forces on members
        if not self.forcesCalced:
            for m in self.members:
                if m.answerForm is False:
                    m.force = m.force * -1
                    m.answerForm = True
                    #print(m.force)
        self.forcesCalced = True
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

    def chooseHSSs(self, HSSList, bottomChordIds, topChordIds, incrChords):
        # Select a member to represent the maximum in each case
        topForce = Member.dummy()
        topLength = Member.dummy()
        botForce = Member.dummy()
        botLength = Member.dummy()
        webForce = Member.dummy()
        webLength = Member.dummy()
        for m in self.members:
            # print(topForce.force)
            placed = False
            for id in bottomChordIds:
                if m.id == id:
                    if botForce.force < abs(m.force):
                        botForce = m.duplicate()
                    if botLength.length < m.length:
                        botLength = m.duplicate()
                    placed = True
                    break
            if placed:
                continue
            for id in topChordIds:
                if m.id == id:
                    if topForce.force < abs(m.force):
                        topForce = m.duplicate()
                    if topLength.length < m.length:
                        topLength = m.duplicate()
                    placed = True
                    break
            if placed:
                continue
            if webForce.force < abs(m.force):
                webForce = m.duplicate()
            if webLength.length < m.length:
                webLength = m.duplicate()
        # Get force signs
        # Set top, bottom and web HSSs
        webMs = []
        anti = True if incrChords is False else False
        for m in self.members:
            set = False
            for id in bottomChordIds:
                if m.id == id:
                    m.pickHSS(HSSList, botForce, botLength, incrChords)
                    chordy = m
                    set = True
                    break
            if set:
                continue
            for id in topChordIds:
                if m.id == id:
                    m.pickHSS(HSSList, topForce, topLength, incrChords)
                    chordx = m
                    set = True
                    break
            if set:
                continue
            # Selects HSS for the web forces
            m.pickHSS(HSSList, webForce, webLength, anti)
            webMs = webMs + [m.id]
            webx = m
        # Check HSS relativity (if chords are smaller than web, increase chords to match web)
        chordSizeT = None
        chordSizeB = None
        if (len(topChordIds) > 0 and topChordIds != bottomChordIds): # Only use bottom chord IDs if they match
            print(bottomChordIds)
            chordSizeT = chordx.structure.size.split('x')
        if (len(bottomChordIds) > 0):
            chordSizeB = chordy.structure.size.split('x')
        webSize = webx.structure.size.split('x')
        # Find Smallest
        targ = None
        # targ = 1
        # fin = 0
        # current = chordSizeT if chordSizeT != None else [0,0,0]
        iter = -1
        fin = 0
        for siz in [chordSizeT, chordSizeB, webSize]:
            iter = iter + 1
            if targ == None:
                targ = siz
                continue
            elif siz == None:
                continue
            elif (float(targ[0]) > float(siz[0])) and (float(targ[2]) > float(siz[2])):
                targ[0] = siz[0]
                targ[2] = siz[2]
                fin = iter
        # Determine smallest members to target
        if fin == 0:
            targetMems = topChordIds
        elif fin == 1:
            targetMems = bottomChordIds
        else:
            targetMems = webMs
        # Add incrementation to smallest
        for m in self.members:
            for id in targetMems:
                m.structureIteration = m.structureIteration + 1
        # Manage Top Chord
        if chordSizeT != None and (float(webSize[0]) > float(chordSizeT[0]) or float(webSize[2]) > float(chordSizeT[2])):
            for m in self.members:
                for id in topChordIds:
                    if m.id == id:
                        # print(m.id)
                        m.pickHSS(HSSList, webForce, webLength, anti)
                        # print(m.structure.size)
        # Manage Bottom Chord
        if chordSizeB != None and (float(webSize[0]) > float(chordSizeB[0]) or float(webSize[2]) > float(chordSizeB[2])):
            for m in self.members:
                for id in bottomChordIds:
                    if m.id == id:
                        m.pickHSS(HSSList, webForce, webLength, anti)
        for m in self.members:
            m.calculateLengthChange()  # this is appropriate as it is calculated based on HSS
        # Calculate total loading with new HSSs
        load = 0
        for m in self.members:
            load = load + m.structure.deadLoad
        return load

    def connectHSSs(self, info):
        for m in self.members:
            for i in info:
                if i[0] == m.id:
                    m.structure = i[1]
                    m.calculateLengthChange()
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
        # print(DOFids)
        super(Joint, self).__init__(point.x, point.y, point.designation)
        self.DOFs = [DOF(DOFids[0]), DOF(DOFids[1])]  # 0 is x, 1 is y
        # print(self.DOFs[0].id)

    def dummy():
        dum = Joint(Point(0, 0), [0, 0])
        dum.setForce('xy', 0)
        dum.setDisp('xy', 0)
        return dum

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
        newJ = Joint(Point(self.x, self.y, self.designation), [dof[0].id, dof[1].id])
        newJ.DOFs[0] = self.DOFs[0].duplicate()
        newJ.DOFs[1] = self.DOFs[1].duplicate()
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


# def getSlideValue(strVal, leng):
#     type = int
#     final = ''
#     x = 0
#     followsPoint = False
#     # Check if there is a decimal in the range
#     for i in range(0, leng, 1):
#         if i >= len(strVal):
#             break # Exit early if done
#         if strVal[i] == '.':
#             leng = leng + 1
#             break
#     for i in range(0, leng, 1):
#         if followsPoint:
#             followsPoint = False
#             continue
#         if i >= len(strVal):
#             return [final, type] # Exit early if done
#         if strVal[i] == '.':
#             final = final + "." + strVal[i + 1]
#             followsPoint = True
#             x = 1
#             type = float
#         else:
#             final = final + strVal[i]
#     i = i + 1
#     if i + 1 >= len(strVal):
#         return [final, type] # Exit early if done now
#     # Check for rounding
#     # Get rounded value
#     z = 1
#     passDec = False
#     l = 0
#     while True:
#         if final[len(final)-z] == '.':
#             z = z + 1
#             passDec = True
#             l = 2
#             continue
#         roundVal = int(final[len(final)-z]) + 1
#         if passDec:
#             l = l + 1
#         if (len(str(roundVal)) > z):
#             z = z + 1
#         else:
#             roundVal = roundVal * mh.pow(10, z-l)
#             break
#     roundVal = int(roundVal)
#     if final[len(final)-z-1] == '.':
#         strip = int(float(final[0:len(final)-z] + str(0)))
#     else:
#         strip = int(float(final[0:len(final)-z]))
#     valsToSize = 0
#     temp = strip
#     if float(final) > 1:
#         while temp < float(final):
#             valsToSize = valsToSize + 1
#             temp = temp * mh.pow(10, valsToSize)
#     # Else we gotta do a decimal thing
#     strip = strip * mh.pow(10, valsToSize-1)
#     if strVal[i] != '.' and int(strVal[i]) > 4:
#         final = strip + roundVal
#     elif strVal[i] == '.' and int(strVal[i+1]) > 4:
#         roundVal = int(roundVal / 10)
#         final = strip + roundVal
#     return [final, type]


def slideRuleAccuracy(value):
    if (value < 1e-12 and value > 0) or value == 0 or (value > -1e-12 and value < 0):
        return 0
    val = value
    # Filter out the negative
    neg = False
    if val < 0:
        val = val * -1
        neg = True
    # Remove trailng zeros in large number
    trailTally = 0
    strVal = str(val)
    init = val
    if val > 1e4:
        # Handle value in scientific form
        sci = 4
        if len(strVal.split("e")) > 1:
            sci = int(strVal.split("e")[1])
            strVal = strVal.split("e")[0] + "0000"
        cut = (strVal[5:len(strVal)]).split(".")[0]
        trailTally = len(cut) + sci - 4
        strVal = strVal[0:5]
        val = int(strVal)
    # Remove leading zeros
    zeroTally = 0
    strVal = str(val)
    if (val < 1 and val > 0):
        # Handle value in scientific form
        sci = 4
        if len(strVal.split("e")) > 1:
            sci = int(strVal.split("e-")[1])
            strVal = "0.000" + strVal.split("e")[0]
        ind = 0
        for num in strVal:
            if num == "0":
                zeroTally = zeroTally + 1
            elif num == ".":
                pass
            else:
                break
            ind = ind + 1
        strVal = strVal[ind:len(strVal)]
        val = float(strVal)
        zeroTally = zeroTally + sci - 4
    # Break and round
    finalLen = 4 if strVal[0] == "1" else 3
    # Make integer for adjustments
    while int(val) != float(val):
        if len(str(int(val))) > 6:
            val = int(str(val).split(".")[0])
            break
        val = val * 10
        zeroTally = zeroTally + 1
    strVal = str(val)
    # Do rounding
    if len(strVal) > finalLen and int(strVal[finalLen]) > 4:
        for pos in range(finalLen - 1, -2, -1):
            if pos == -1:
                strVal = "1" + strVal
                break
            if int(strVal[pos]) == 9:
                strVal = strVal[0:pos] + "0" + strVal[pos+1:len(strVal)]
            else:
                strVal = strVal[0:pos] + str(int(strVal[pos]) + 1) + strVal[pos+1:len(strVal)]
                break
    val = float(strVal[0:finalLen])
    # Check for removed places
    testStr = str(val).split(".")[0]
    if (len(testStr) < len(strVal)) and init > 1:
        trailTally = trailTally + len(strVal) - len(testStr)
    # Adjust for added places
    if int(str(val).split(".")[1]) > 0:
        zeroTally = zeroTally + finalLen - 1 if finalLen < len(strVal) else zeroTally + len(strVal) - 1
    proper = val
    proper = float(str(proper) + "e" + str(trailTally))
    proper = float(str(proper) + "e-" + str(zeroTally))
    # Cut value again for problems with pow
    return proper if not neg else -proper


# def slideRuleAccuracy(value):
#     # No need to convert a 0
#     val = value
#     if val == 0:
#         return 0
#     conv = False
#     if val < 0.0000000001 and val > 0:
#         val = val * 10000000000
#         conv = True
#     type = int
#     strVal = str(val)
#     is0Dec = False
#     neg = False
#     if (val < 0):
#         strVal = strVal[1:len(strVal)]
#         neg = True
#     if (val < 1) and (val > -1):
#         type = float
#         strVal = strVal[2:len(strVal)] # remove the 0.
#         is0Dec = True
#     # Remove leading 0s
#     zeros = ''
#     for v in strVal:
#         if v == '0':
#             zeros = zeros + '0'
#         else:
#             break
#     strVal = strVal[len(zeros):len(strVal)]
#     # Use slide rule on sig digs
#     if (strVal[0] == '1'):
#         res = getSlideValue(strVal, 4)
#     else:
#         res = getSlideValue(strVal, 3)
#     final = res[0]
#     if res[1] == float:
#         type = float
#     # Add values back
#     if is0Dec:
#         final = "0." + zeros + str(final)
#     if len(str(final).split('.')) > 2:
#         f = str(final).split('.')[0:2]
#         finalS = ""
#         for i in f:
#             finalS = finalS + i
#         final = float(finalS)
#     # Cast to Type
#     if type == int:
#         result = int(final)
#     else:
#         result = float(final)
#     if conv:
#         result = result / 10000000000
#     if neg:
#         return -result
#     else:
#         return result


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
    return True

# test()
