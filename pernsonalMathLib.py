def translateMathIndexToCode(ind):
    if ind > 0:
        return ind - 1
    else:
        return ind

class Matrix:
    def __init__(self, inList):
        # inList should be a list of lists representing the data structure
        # Process list and check length of rows
        if type(inList) != list:
            raise ValueError("Input value must be a list")
        
        self.rows = len(inList)
        self.cols = 0
        
        self.valType = int # Default expected datatype
        
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
        
        #Process and create the matrix construct
        self.M = []
        # Generate the matrix empty
        for r in range(0, self.rows, 1):
            self.M = self.M + [[]] # Defines a new row
            for c in range(0, self.cols, 1):
                self.M[r] = self.M[r] + [0]
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
        
    def checkDataType(self, val):
        try:
            temp = float(val)
            # Check if float is required
            if temp != int(val):
                self.setDataType("float")
            return True
        except:
            return False
            
    def cleanData(self, val):
        if self.valType == float:
            return float(val)
        elif self.valType == int:
            return int(val)
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
            self.recalculate()
            
    def recalculate(self):
        for r in range(0, self.rows, 1):
            for c in range(0, self.cols, 1):
                self.M[r][c] = self.cleanData(self.M[r][c])
    
    # User speaks math --> Row index 0 is row 1 to user (stakeholder evaluation)
        
    def getValue(self, row, col):
        # r = translateMathIndexToCode(row)
        # c = translateMathIndexToCode(col)
        if (r < self.rows) and (c < self.cols):
            return self.M[r][c]
        else:
            return False
            
    def setValue(self, row, col, val):
        if (row < self.rows) and (col < self.cols):
            self.M[row][col] = self.cleanData(val)
            return True
        else:
            print("WARNING: The value could not be set as this position does not exist in the matrix")
            return False
    
    def getRow(self, row):
        # r = translateMathIndexToCode(row)
        if r < self.rows:
            return self.M[r]
        else:
            return False
            
    def setRow(self, row, inRow):
        # r = translateMathIndexToCode(row)
        if r < self.rows:
            if len(inRow) == self.cols:
                # Default col index
                c = 0
                for val in inRow:
                    self.M[r][c] = self.cleanData(val)
                    c = c + 1
                return True
            else:
                return False
        else:
            print("WARNING: The values could not be set as this row does not exist in the matrix")
            return False
            
    def display(self):
        printStr = ""
        for row in self.M:
            for val in row:
                printStr = printStr + format(val, '.4g') + " " # Had to look up the g vs f from what we've seen :(
            printStr = printStr + "\n"
        print(printStr)
        return True
        
    def setDims(rows, columns):
        x = []
        for r in range(0, rows, 1):
            x = x + [[]] # Defines a new row
            for c in range(0, columns, 1):
                x[r] = x[r] + [0] # Add a value to the row to generate column length
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
                    finalM.setValue(r, c, val)
            return finalM
        else:
            raise ValueError("Cannot multiply these matricies")
            
    def scale(self, scalar):
        if not self.checkDataType(scalar):
            return False
        for r in range(0, self.rows, 1):
            for c in range(0, self.cols, 1):
                self.setValue(r, c, scalar*self.M[r][c])
        return True
    
    def invert(self):
        if self.rows != self.cols:
            return False
        #Get the determinant
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
        self.cofactor() # Adjugate 
        self.transpose() # GOTTA DEFINE DAT
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
        
    def cofactor(self): # Only for square matricies
        if self.rows != self.cols:
            return False
        # If its has a determinant, compute!
        if self.rows == 2:
            dup = self.M
            self.M[0][0] = dup.M[1][1]
            self.M[0][1] = -dup.M[0][1]
            self.M[1][0] = -dup.M[1][0]
            self.M[1][1] = dup.M[0][0]
            print(self.M)
            return True
        else:
            dup = self.duplicate()
            for r in range(0, self.rows, 1):
                targetRows = self.M[0:r] + self.M[r+1:self.rows]
                #print(targetRows)
                for c in range(0, self.cols, 1):
                    # Generate mini target list
                    miniM = []
                    # Remove the column that is related to the current column
                    for row in targetRows:
                        tempR = []
                        tempR = tempR + row[0:c]
                        tempR = tempR + row[c+1:self.cols]
                        miniM = miniM + [tempR]
                    print(miniM)
                    if (c+r) % 2 == 0:
                        val = Matrix(miniM).getDeterminant()
                    else:
                        val = -Matrix(miniM).getDeterminant()
                    dup.M[r][c] = val
                self.M = dup.M
            return True
    
    def duplicate(self):
        return self
        
    def getDeterminant(self):
        if self.rows != self.cols:
            return False
        # If is has a determinant, compute!
        if self.rows == 2:
            return self.M[0][0]*self.M[1][1] - self.M[0][1]*self.M[1][0]
        else:
            targetRows = self.M[1:self.rows]
            det = 0
            for i in range(0, self.cols, 1):
                # Generate mini target list
                miniM = []
                # Remove the column that is related to the current column
                for row in targetRows:
                    tempR = []
                    tempR = tempR + row[0:i]
                    tempR = tempR + row[i+1:self.cols]
                    miniM = miniM + [tempR]
                # Get determinant using mini lists
                tempDet = self.M[0][i]*Matrix(miniM).getDeterminant()
                if i % 2 == 0:
                    det = det + tempDet
                else:
                    det = det - tempDet
            return det
    
def Test():
    x = Matrix([[1,1], [1,2], [1,5]])
    x.display()
    z = Matrix([1])
    y = Matrix([[3,1,1,7], [4,2,2,6], [9,4,1,8], [1,9,4,5]])
    y.display()
    y.invert()
    y.display()
    
Test()
