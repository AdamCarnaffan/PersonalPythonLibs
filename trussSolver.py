# Inspired by https://nptel.ac.in/courses/Webcourse-contents/IIT%20Kharagpur/Structural%20Analysis/pdf/m4l25.pdf

from civLib import *
import sys


class Load:

    def __init__(self, loads):
        self.live = loads[0]
        self.deck = loads[1]
        self.member = loads[2]
        self.area = loads[3]
        self.calculateTotal()

    def setMemberLoad(self, val):
        self.member = val
        return True

    def calculateTotal(self):
        self.total = (self.member + self.deck + self.live)*self.area
        return True

    def setTotalLoad(self, val):
        self.total = val
        return True

    def getTotal(self):
        return self.total

    def getSelfWeight(self):
        return self.member + self.deck

    def getLiveWeight(self):
        return self.live

    def resetMemberLoad(self, load):
        self.member = load
        self.calculateTotal()
        return True

    def duplicate(self):
        new = Load([self.live, self.deck, self.member])
        return new


class Options:

    def __init__(self):
        self.isBridge = False

    def setBridge(self, boo):
        self.isBridge = boo

    def getIsBridge(self):
        return self.isBridge


def stripZeros(l):
    final = []
    for val in l:
        if int(val) != 0:
            final = final + [val]
    return final


def main():
    # Read HSSs into a selection array
    inp = open("inputs.txt", 'r')
    data = inp.readlines()
    mode = 0
    # Set Truss Defaults
    vars = []
    points = []
    members = []
    memberIndex = 0
    restrict = []
    force = []
    pointOfDeflection = 0
    deflectionSpan = []
    loads = [0, 0, 0, 0]  # Set defaults for all loads
    d = 0
    options = Options()
    for l in data:
        data[d] = l.split("\n")[0]
        d = d + 1
    for line in data:
        # Detect comments
        if line[0] == '#':
            continue
        # Detect new mode
        if line == "Variables":
            mode = 1
        elif line == "Points":
            mode = 2
        elif line == "Members":
            mode = 3
        elif line == "Restraints":
            mode = 4
        elif line == "Forces":
            mode = 5
        elif line == "Point of Deflection":
            mode = 6
        elif line == "Span of Deflection":
            mode = 7
        elif line == "Bridge Loads":
            mode = 8
        elif line == "Options":
            mode = 9
        else:  # If no mode setting
            cur = line.split(" ")
            if mode != 1:
                dataVals = []
                for l in cur:
                    dataVals = dataVals + [interpVar(l, vars)]
            # Action based on mode
            if mode == 1:
                vars = vars + [Variable(cur[0], cur[1])]
            elif mode == 2:
                points = points + [Point(dataVals[0], dataVals[1])]
            elif mode == 3:
                memberIndex = memberIndex + 1
                members = members + [Member(dataVals[0], dataVals[1], memberIndex)]
            elif mode == 4:
                restrict = restrict + [[dataVals[0], dataVals[1]]]
            elif mode == 5:
                force = force + [[dataVals[0], dataVals[1], dataVals[2]]]
            elif mode == 6:
                pointOfDeflection = dataVals[0]
            elif mode == 7:
                deflectionSpan = [dataVals[0], dataVals[1]]
            elif mode == 8:
                if dataVals[0] == "Live":
                    loads[0] = dataVals[1]
                elif dataVals[0] == "Deck":
                    loads[1] = dataVals[1]
                elif dataVals[0] == "Truss":
                    loads[2] = dataVals[1]
                elif dataVals[0] == "TribArea":
                    loads[3] = dataVals[1]
            elif mode == 9:
                if dataVals[0] == "isBridge":
                    if dataVals[1] == 1:
                        options.setBridge(True)
    # Generate all other dofs as points of 0 force
    sets = restrict + force
    for u in range(1, len(points) + 1, 1):
        offsetStr = ''
        for r in sets:
            if r[0] == u:
                offsetStr = offsetStr + r[1]
        if offsetStr == 'yx':
            offsetStr = 'xy'
        finalStr = subtract('xy', offsetStr)
        if finalStr != '':
            force = force + [[u, finalStr]]
    # points = [Point(0,0), Point(2.75, h), Point()]
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
        joints = joints + [Joint(p, [(i*2)-1, i*2])]
        i = i + 1
    # members = [Member(1,2,1), Member(1,3,2), Member(2,3,3), Member(2,4,4), Member(3,4,5), Member(3,5,6), Member(4,5,7)]
    # restrict = [[1, 'xy'], [5, 'y']]  # Disp is set 0
    # force = [[2, 'xy'], [3, 'x'], [3, 'y', -20], [4, 'xy'], [5, 'x']]  # Forces are set (Default val is 0 when not specified)
    # pointOfDeflection = 3
    # deflectionSpan = [1,5] # Members from and to
    loading = Load(loads)
    toggleUptick = True
    isEstimation = True
    established = False
    fullPass = False
    prevState = []
    while True:
        if established:
            state = []
            for m in truss.getMembers():
                state = state + [[m.getID(), m.getIteration()]]
        truss = Truss(joints, members)
        bottomSpanLength = points[deflectionSpan[0]-1].distance(points[deflectionSpan[1]-1])
        for r in restrict:
            truss.fetchJoint(r[0]).setDisp(r[1], 0)
        # Determine forces using Load variable
        if options.getIsBridge():
            # Get joints to distribute between
            mems = truss.selectSpan('lower')
            # Get joints from members in span
            join = []
            for l in mems:
                m = truss.getMemberByID(l)
                tempNew = [m.a, m.b]
                for t in tempNew:
                    found = False
                    for j in join:
                        if j == t:
                            found = True
                    if not found:
                        join = join + [t]
            # Calculate joints to distribute load (Remove joints that are restricted)
            for d in restrict:
                id = 0
                for j in join:
                    if j == str(d[0]):
                        join[id] = '0'
                    id = id + 1
            lowerPoints = stripZeros(join)
            loadPerPoint = slideRuleAccuracy(loading.getTotal())
            ind = 0
            for f in force:
                for j in lowerPoints:
                    if str(f[0]) == j:
                        # Manipulate joint
                        if f[1] == 'xy':
                            # Split x and y
                            force = force + [[f[0], 'x']]
                            f[1] = 'y'
                        elif f[1] == 'x':
                            continue
                        if len(f) == 2:
                            force[ind] = force[ind] + [0]
                        # Now we have y for sure
                         # Y is negative for gravity
                        force[ind][2] = -loading.getTotal()
                ind = ind + 1
        for f in force:
            val = 0 if len(f) == 2 else f[2]
            truss.fetchJoint(f[0]).setForce(f[1], val)
        truss.calculateDisplacements()
        truss.calculateJointForces()
        truss.calculateMemberForces()
        # print(truss.joints[0].getDOF(2).force)
        truss.getAnswerForces()
        # Assign Estimated Truss for HSSs
        if (isEstimation):
            estTruss = truss
            isEstimation = False # Only first pass is load estimation point
        # Apply structural iterations
        # if established:
        #     truss.connectHSSIterations(state)
        # l = truss.getMemberByID(2)
        # print(l.jointA.x, l.jointA.y)
        # print(l.jointB.x, l.jointB.y)
        # print(l.force)
        # break
        # truss.display()
        # sys.exit()
        # Build list of HSSs
        HSSFile = open('fixedData\\HSSData.txt', 'r')
        HSSData = HSSFile.readlines()
        HSSs = []
        for line in HSSData:
            HSSs = HSSs + [HSS(line)]
        # prevLoad = loading.getTotal()
        print("STATE IS", fullPass)
        if not fullPass:
            loading.resetMemberLoad(estTruss.chooseHSSs(HSSs, estTruss.selectSpan('lower'), estTruss.selectSpan('height'), toggleUptick))
            # Apply to current truss
            structure = []
            for m in estTruss.members:
                structure = structure + [[m.id, m.structure]]
            truss.connectHSSs(structure)
        else:
            truss.connectHSSs(prevState)
        print("Picked HSSs")
        # break
        # b = truss.getMemberByID(1)
        # b.dispMatrix.display()
        # b.stiffnessMatrix.display()
        # s = truss.getMemberByID(3)
        # s.jointA.display()
        # s.jointB.display()
        # print(s.cosine)
        # print(s.sine)
        # s.dispMatrix.display()
        # s.stiffnessMatrix.display()
        # truss.stiffness.display()
        # sys.exit()
        # truss.display()
        established = True  # For maintaining HSS establishment state
        # toggleUptick = False if toggleUptick else True
        # truss.display()
        # truss.display()
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
                    # Not completely expandable
                    val = -1
            virtualTruss.fetchJoint(f[0]).setForce(f[1], val)
        # for j in virtualTruss.joints:
        #     print(j.DOFs[0].disp)
        virtualTruss.calculateDisplacements()
        virtualTruss.calculateJointForces()
        virtualTruss.calculateMemberForces()
        # print(truss.joints[0].getDOF(2).force)
        virtualTruss.getAnswerForces()
        # virtualTruss.display()
        # 0.0071784033525843245
        visualDisp = abs(truss.getDeltaR(virtualTruss))/(bottomSpanLength)
        if visualDisp > 1/350:
            # Must recalculate
            print("Failed Visual", visualDisp)
            fullPass = False
            continue
        print(visualDisp)
        # Do frequency stuff
        # Create a truss with the oscillatory load
        distrib = 17.75
        selfFreq = distrib/mh.sqrt(loading.getSelfWeight())
        if selfFreq < 2.1 and selfFreq > 1.9:
            # Must recalculate
            fullPass = False
            continue
        maxFreq = 2.0
        DAF = 1/(1+(maxFreq/selfFreq))
        # maxTruss = Truss(joints, members)
        # for f in force:
        #     val = 0 if len(f) == 2 else f[2]
        #     maxTruss.fetchJoint(f[0]).setForce(f[1], val*1.5)
        # for r in restrict:
        #     maxTruss.fetchJoint(r[0]).setDisp(r[1], 0)
        # maxTruss.calculateDisplacements()
        # maxTruss.calculateJointForces()
        # maxTruss.calculateMemberForces()
        # # print(truss.joints[0].getDOF(2).force)
        # maxTruss.getAnswerForces()
        # maxTruss.chooseHSSs(HSSs, maxTruss.selectSpan('lower'), maxTruss.selectSpan('upper'), True)
        df = 0.5*(loading.getLiveWeight()/loading.getSelfWeight())*visualDisp
        maxDisp = visualDisp + (DAF*df)
        if maxDisp > 1/350:
            # Must recalculate
            fullPass = False
            continue
        # print(maxDisp)
        if fullPass:
            break
        fullPass = True
        print("Made full pass")
        truss.display()
        # Save previous HSSs for verification
        prevState = []
        for m in truss.members:
            prevState = prevState + [[m.id, m.structure]]
    print("-----RESULT-----")
    cost = 0
    # Compute Cost
    for m in truss.members:
        if m.structure.isCustom:
            mn = m.structure.deadLoad / 9.81
            cost = cost + mn*m.length*4
        else:
            cost = cost + m.structure.mass*m.length*3
    for j in truss.joints:
        cost = cost + 750
    # Get Fixed Forces
    truss.compileDofs()
    restrictedDofs = []
    for point in restrict:
        if point[1] == 'x':
            restrictedDofs = restrictedDofs + [(int(point[0])*2) - 1]
        elif point[1] == 'y':
            restrictedDofs = restrictedDofs + [(int(point[0])*2)]
        else:
            restrictedDofs = restrictedDofs + [(int(point[0])*2) - 1]
            restrictedDofs = restrictedDofs + [(int(point[0])*2)]
    for d in truss.DOFs:
        for r in restrictedDofs:
            if r == d.id:
                print("DOF", d.id, "=>", slideRuleAccuracy(d.force))
    print("LOAD OF", str(loading.getTotal()) + "kN (per loading point)")
    print("TOTAL COST OF $" + str(cost))
    truss.display()
    return True

main()
