import math as mh


def translateMathIndexToCode(ind):
    if ind > 0:
        return ind - 1
    else:
        return ind


class Matrix:
    def __init__(self, inList, rowLabels=[], colLabels=[]):
        # inList should be a list of lists representing the data structure
        # Process list and check length of rows
        if type(inList) != list:
            raise ValueError("Input value must be a list")
        self.rows = len(inList)
        self.cols = 0
        self.valType = int  # Default expected datatype

        # Process and create the matrix construct
        self.M = []
        self.rowLabels = []
        self.colLabels = []

        # Scan list for expected col length
        for row in inList:
            if type(row) != list:
                if self.checkDataType(row):
                    self.cols = 1
                    continue
                else:
                    raise ValueError("The data in a matrix must be numeric")
            elif len(row) < self.cols or self.cols == 0:
                self.cols = len(row)
        # Generate the matrix empty
        for r in range(0, self.rows, 1):
            self.M = self.M + [[]]  # Defines a new row
            self.rowLabels = self.rowLabels + [""]  # Add a new row label for the row
            for c in range(0, self.cols, 1):
                if r == 0:
                    self.colLabels = self.colLabels + [""]  # Add a new col label for the column
                self.M[r] = self.M[r] + [0]
        # Add labels in rows and columns starting from left/top going to last available
        i = 0
        for label in rowLabels:
            if i >= self.rows:
                break
            self.rowLabels[i] = label
            i = i + 1
        i = 0
        for label in colLabels:
            if i >= self.cols:
                break
            self.colLabels[i] = label
            i = i + 1
        # Type the empty matrix properly
        self.recalculate()
        # Default iterables for rows and cols
        tempRInd = 0
        tempCInd = 0
        # Check and add the values
        for row in inList:
            tempCInd = 0
            if type(row) != list:
                self.M[tempRInd][tempCInd] = self.cleanData(row)
            else:
                for val in row:
                    if self.checkDataType(val):
                        self.M[tempRInd][tempCInd] = self.cleanData(val)
                    else:
                        raise ValueError("The data in a matrix must be numeric")
                    tempCInd = tempCInd + 1
                tempRInd = tempRInd + 1

    def setRowLabels(self, rowLabels):
        i = 0
        for label in rowLabels:
            self.rowLabels[i] = label
            i = i + 1
            if i >= self.rows:
                break
        return True

    def setColLabels(self, colLabels):
        i = 0
        for label in colLabels:
            if i >= self.cols:
                break
            self.colLabels[i] = label
            i = i + 1
        return True

    def getRowByLabel(self, rowL):
        i = 0
        for r in self.rowLabels:
            if rowL == r:
                return self.M[i]
            i = i + 1
        return False

    def getIndexByLabel(self, label, dir):
        ls = self.rowLabels if dir == 'row' else self.colLabels
        i = 0
        for l in ls:
            if l == label:
                return i
            i = i + 1
        return False

    def getValuebyLabels(self, rowL, colL):
        foundR = False
        val = 0
        index = 0
        for row in self.rowLabels:
            if row == rowL:
                rowList = self.M[index]
                foundR = True
                break
            index = index + 1
        if not foundR:
            return False
        foundL = False
        index = 0
        for col in self.colLabels:
            if col == colL:
                val = rowList[index]
                foundL = True
                break
            index = index + 1
        if not foundL:
            return False
        return val

    def checkDataType(self, val):
        try:
            #print(val)
            temp = float(val)
            # Check if float is required
            if temp != int(val):
                #print("hey")
                self.setDataType("float")
            return True
        except:
            return False

    def cleanData(self, val):
        finalVal = val
        if (val < 0.00000000000001 and val > 0) or (val > -0.00000000000001 and val < 0):
            finalVal = 0
            pass
        if self.valType == float:
            return float(finalVal)
        elif self.valType == int:
            return int(finalVal)
        else:
            return False

    def setDataType(self, typeStr):
        prevType = self.valType
        if typeStr == "float":
            self.valType = float
        elif typeStr == "int":
            self.valType = int
        # Re-calculate all values if not same type as before
        if prevType != self.valType:
            if len(self.M) > 0:
                self.recalculate()
        return True

    def recalculate(self):
        for r in range(0, self.rows, 1):
            for c in range(0, self.cols, 1):
                self.M[r][c] = self.cleanData(self.M[r][c])
        return True

    # User speaks math --> Row index 0 is row 1 to user (stakeholder evaluation)

    def getValue(self, row, col):
        # r = translateMathIndexToCode(row)
        # c = translateMathIndexToCode(col)
        if (row < self.rows) and (col < self.cols):
            return self.M[row][col]
        else:
            return False

    def setValue(self, row, col, val, overrideType = False):
        if (row < self.rows) and (col < self.cols):
            if overrideType:
                self.checkDataType(val)
            self.M[row][col] = self.cleanData(val)
            return True
        else:
            print("WARNING: The value could not be set as this position does not exist in the matrix")
            return False

    def setValueByLabels(self, rowL, colL, val, overrideType = False):
        if self.checkLabels([rowL], 'row') and self.checkLabels([colL], 'col'):
            if overrideType:
                self.checkDataType(val)
            self.setValue(self.getIndexByLabel(rowL, 'row'), self.getIndexByLabel(colL, 'col'), val, True)
            return True
        else:
            print("WARNING: The value could not be set as this position does not exist in the matrix")
            return False

    def getRow(self, row):
        # r = translateMathIndexToCode(row)
        result = []
        if row < self.rows:
            for val in self.M[row]:
                result = result + [val]
            return result
        else:
            return False

    def setRow(self, row, inRow):
        # r = translateMathIndexToCode(row)
        if row < self.rows:
            if len(inRow) == self.cols:
                # Default col index
                c = 0
                for val in inRow:
                    self.M[row][c] = self.cleanData(val)
                    c = c + 1
                return True
            else:
                return False
        else:
            print("WARNING: The values could not be set as this row does not exist in the matrix")
            return False

    def getRowLabel(self, row):
        return self.rowLabels[row]

    def getColLabel(self, col):
        return self.colLabels[col]

    def getLabels(self, select):
        if select == "row":
            return self.rowLabels
        elif select == "column":
            return self.colLabels
        else:
            return False

    def display(self):
        printStr = ""
        for row in self.M:
            for val in row:
                printStr = printStr + format(val, '.8g') + " " # Had to look up the g vs f from what we've seen :(
            printStr = printStr + "\n"
        print(printStr)
        return True

    def setDims(rows, columns):
        x = []
        for r in range(0, rows, 1):
            x = x + [[]]  # Defines a new row
            for c in range(0, columns, 1):
                x[r] = x[r] + [0]  # Add a value to the row to generate column length
        return Matrix(x)

    def __add__(self, other):
        if (self.rows == other.rows) and (self.cols == other.cols):
            finalM = Matrix.setDims(self.rows, self.cols)
            for r in range(0, self.rows, 1):
                for c in range(0, self.cols, 1):
                    finalM.setValue(r, c, self.M[r][c] + other.M[r][c])
            return finalM
        else:
            raise ValueError("Matricies must be of the same dimmensions to add")

    def __mul__(self, other):
        if (self.cols == other.rows):
            finalM = Matrix.setDims(self.rows, other.cols)
            for r in range(0, self.rows, 1):
                for c in range(0, other.cols, 1):
                    # Set a value at each position of finalM.M here
                    val = 0
                    for i in range(0, self.cols, 1):
                        val = val + (self.M[r][i]*other.M[i][c])
                    finalM.setValue(r, c, val, True)
            return finalM
        else:
            raise ValueError("Cannot multiply these matricies")

    def checkLabels(self, labels, dir):
        ls = self.rowLabels if dir == 'row' else self.colLabels
        result = []
        for l in labels:
            found = False
            for la in ls:
                if str(l) == str(la):
                    found = True
                    break
            if not found:
                return False
        return True

    def getSubByLabels(self, rowLabels, colLabels):
        final = Matrix.setDims(len(rowLabels), len(colLabels))
        final.setRowLabels(rowLabels)
        final.setColLabels(colLabels)
        final.setDataType(self.valType)
        # Check row & col labels exist in labels
        if not self.checkLabels(rowLabels, 'row') or not self.checkLabels(colLabels, 'col'):
            return False
        # Compile rows
        rows = []
        for r in rowLabels:
            rows = rows + [Matrix(self.getRowByLabel(r), [r], self.colLabels)]
        # Slice rows by col ids
        for m in rows:
            for l in colLabels:
                val = self.getValuebyLabels(m.rowLabels[0], l)
                s = final.setValueByLabels(m.rowLabels[0], l, val, True)
        return final

    def scale(self, scalar):
        if not self.checkDataType(scalar):
            return False
        for r in range(0, self.rows, 1):
            for c in range(0, self.cols, 1):
                self.setValue(r, c, scalar*self.M[r][c])
        return True

    def returnScale(self, scalar):
        dup = self.duplicate()
        dup.scale(scalar)
        return dup

    def invert(self):
        if self.rows != self.cols:
            return False
        # Get the determinant
        determinant = self.getDeterminant()
        if determinant == 0:
            return False
        # Get the adjusted position matrix
        adjusted = self.duplicate()
        adjusted.adjugate()
        # apply determinant to adjusted
        adjusted.scale(1/determinant)
        # Set adjusted to self
        self.M = adjusted.M
        return True

    def adjugate(self):
        self.cofactor()
        self.transpose()
        return True

    def transpose(self):
        old = self.duplicate()
        # Update row and columns count
        self.rows = old.cols
        self.cols = old.rows
        # Create dummy to hold matrix data
        new = Matrix.setDims(self.rows, self.cols)
        # Set Matrix data from old
        for r in range(0, old.rows, 1):
            for c in range(0, old.cols, 1):
                new.M[c][r] = old.M[r][c]
        # Reflect onto original
        self.M = new.M
        return True

    def cofactor(self):  # Only for square matricies
        if self.rows != self.cols:
            return False
        # If its has a determinant, compute!
        if self.rows == 2:
            dup = self.duplicate()
            self.M[0][0] = dup.M[1][1]
            self.M[0][1] = -dup.M[0][1]
            self.M[1][0] = -dup.M[1][0]
            self.M[1][1] = dup.M[0][0]
            # print(self.M)
            return True
        else:
            dup = self.duplicate()
            # print(self.M)
            for r in range(0, self.rows, 1):
                targetRows = self.M[0:r] + self.M[r+1:self.rows]
                # print(r, targetRows)
                # print(self.M)
                # print(targetRows)
                for c in range(0, self.cols, 1):
                    # Generate mini target list
                    miniM = []
                    # Remove the column that is related to the current column
                    for row in targetRows:
                        tempR = []
                        tempR = tempR + row[0:c]
                        tempR = tempR + row[c+1:self.cols]
                        miniM = miniM + [tempR]
                    # if (r == 0) and c == 1:
                    #     x = Matrix(miniM)
                    #     x.display()
                    #     print(x.M)
                    #     print(x.getDeterminant())
                    if (c+r) % 2 == 0:
                        val = Matrix(miniM).getDeterminant()
                    else:
                        val = -Matrix(miniM).getDeterminant()
                    # print("BEFORE: ", self.M)
                    dup.M[r][c] = val
                    # print("AFTER: ", self.M)
            self.M = dup.M
            return True

    def duplicate(self):
        new = Matrix.setDims(self.rows, self.cols)
        # To avoid list with same pointer? (my theory)
        for r in range(0, new.rows, 1):
            new.M[r] = list(self.M[r])
        new.valType = self.valType
        return new

    def modifyRowAdd(self, row, otherRow):
        for c in range(0, self.cols, 1):
            self.setValue(row, c, self.getValue(row, c)+otherRow[c])
        return True

    def swapRows(self, r1, r2):
        row1 = self.getRow(r1)
        row2 = self.getRow(r2)
        self.setRow(r2, row1)
        self.setRow(r1, row2)
        return True

    def getREF(self):
        # dup = self.duplicate()
        # start = 1
        # swaps = 0
        # for c in range(0, dup.cols-1, 1):
        #     i = 0
        #     while True:
        #         if dup.M[start - 1][c] == 0:
        #             if (start + i >= self.rows):
        #                 return c # Rows = cols
        #             dup.swapRows(start-1, start + i)
        #             swaps = swaps + 1
        #             i = i + 1
        #         else:
        #             break
        #     row = Matrix([dup.M[start - 1]])
        #     for r in range(start, dup.rows, 1):
        #         currentValue = dup.getValue(r, c)
        #         rowDupe = row.duplicate()
        #         rowDupe.scale(-(currentValue/rowDupe.M[0][c]))
        #         dup.modifyRowAdd(r, rowDupe.M[0])
        #     start = start + 1
        # dup.swaps = swaps
        dup = self.duplicate()

        return dup

    def geFwdStep(self):
        # Make sure first is non-zero row
        if self.M[0][0] == 0:
            ind = 0
            for r in self.M:
                if r[0] != 0:
                    self.swapRows(0, ind)
                ind = ind + 1
        # Add multiples of first to lowers
        first = self.M[0]
        for r in range(1, self.rows, 1):
            # Scale row
            sc = Matrix([first])
            sc.scale(-(self.M[r][0]/sc.M[0][0])) # For subtraction
            rowM = Matrix([self.M[r]])
            finalRow = rowM + sc


    def getDeterminant(self):
        if self.rows != self.cols:
            return False
        # If is has a determinant, compute!
        if self.rows == 2:
            return self.M[0][0]*self.M[1][1] - self.M[0][1]*self.M[1][0]
        else:
            i = 0
            while True:
                ref = self.getREF()
                if type(ref) == int:
                    self.display()
                    break
                    self.swapRows(i, ref)
                    i = i + 1
                    continue
                break
            det = 1
            for r in range(0, ref.rows, 1):
                det = det * ref.getValue(r, r)
            if ref.swaps % 2 != 0:
                det = det * -1
            return det
            # targetRows = self.M[1:self.rows]
            # det = 0
            # for i in range(0, self.cols, 1):
            #     # Generate mini target list
            #     miniM = []
            #     # Remove the column that is related to the current column
            #     for row in targetRows:
            #         tempR = []
            #         tempR = tempR + row[0:i]
            #         tempR = tempR + row[i+1:self.cols]
            #         miniM = miniM + [tempR]
            #     # Get determinant using mini lists
            #     tempDet = self.M[0][i]*Matrix(miniM).getDeterminant()
            #     if i % 2 == 0:
            #         det = det + tempDet
            #     else:
            #         det = det - tempDet
            # return det


class Vector:

    def __init__(self, x, y):
        pass


class Point:

    def __init__(self, x, y, des=""):
        self.x = x
        self.y = y
        self.designation = des  # ID of the point (can be letter or number)

    def distance(self, other):  # Other is another point
        tempX = mh.pow(self.x - other.x, 2)
        tempY = mh.pow(self.y - other.y, 2)
        return mh.sqrt(tempX + tempY)

    def getName(self):
        return self.designation

    def setName(self, name):
        self.designation = name
        return True

    def display(self):
        print("({0}, {1})".format(self.x, self.y))
        return True


def Test():
    x = Matrix([[0.280247950437196, -0.0615670051298918, 0.08886431746235528, -0.18181818181818182, 0.0, 0.0, 0.0], [-0.0615670051298918, 0.5115670051298918, -0.002261777083911398, -0.05, -0.08660254037844388, -0.2, 0.0], [0.08886431746235528, -0.002261777083911398, 0.278264594020608, -0.08660254037844388, -0.15000000000000002, 0.0, 0.0], [-0.18181818181818182, -0.05, -0.08660254037844388, 0.2818181818181818, 0.0, -0.05, 0.08660254037844388], [0.0, -0.08660254037844388, -0.15000000000000002, 0.0, 0.30000000000000004, 0.08660254037844388, -0.15000000000000002], [0.0, -0.2, 0.0, -0.05, 0.08660254037844388, 0.25, -0.08660254037844388], [0.0, 0.0, 0.0, 0.08660254037844388, -0.15000000000000002, -0.08660254037844388, 0.15000000000000002]])
    x.invert()
    x.display()

#Test()
