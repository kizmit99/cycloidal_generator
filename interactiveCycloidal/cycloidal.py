"""Create cycloidal gearbox"""

import adsk.core
import adsk.fusion
import traceback
import math
from . import fusionUtils


def run(context):
    """ The function that is run by Fusion """

    default_name = 'Cycloidal-custom' # The name which appears in the top bar
    parameters = fusionUtils.Parameters()

    """Parameters to appear in the Fusion window
        Parameters will apear in the order here with the following values:
        name: the varuable name that will hold the valie
        units: the units that the value will be converted to. "" for unitless
        description: the text which will appear with the box
        default_value: the initial value that will appear before being edited """

    parameters.addParameter('fixedPinRingDiameter', "mm", 'Fixed Pin Ring Diameter', 5)
    parameters.addParameter('N', "", 'Number of Fixed Pins', 10)
    parameters.addParameter('fixedPinDiameter', "mm", 'Diameter of Fixed Pins', .3)
    parameters.addParameter('rotorThickness', "mm", 'Rotor Thickness', .5)
    parameters.addParameter('bore', "mm", 'Bore Diameter', 1)
    parameters.addParameter('numGears', "", 'Number of Gears', 1)
    parameters.addParameter('numHoles', "", 'Number of Drive Holes', 0)
    parameters.addParameter('holeCircleDiameter', "mm", 'Diameter of Hole Circle', 3)
    parameters.addParameter('holePinDiameter', "mm", 'Diameter of Drive Pins', .25)
    parameters.addParameter('eccentricityRatio', "", 'Eccentricity Ratio', .8)

    created_object = CreatedObject() # Create an instance of the designed class
    fusionUtils.run(parameters, default_name, created_object)


class CreatedObject:
    """ The class which contains definitions to create the part """

    def __init__(self):
        self.parameters = {}

    def buildRotor(self, app, ui, index, D, N, Dp, rotorThickness, bore, numHoles, holeCircleDiameter, holePinDiameter, eccentricityRatio):
        #other constants based on the original inputs
        d = (float(N - 1) / float(N)) * D #cycloid base circle diameter
        sigma = D / float(N) #rolling circle diameter
        E = eccentricityRatio * (sigma / 2.0) #eccentricity

        zPos = index * rotorThickness

        if ((index % 2) == 0):
            rotorOffset = -E
            rotorRotation = 0
            holeRotation = 0
        else:
            rotorOffset = E
            rotorRotation = math.pi / N
            holeRotation = -rotorRotation

        maxDist = 0.25 * (Dp / 2.0) #maximum allowed distance between points
        minDist = 0.5 * maxDist #the minimum allowed distance between points
         
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        root = design.rootComponent

        rotorOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        rotor = rotorOcc.component
        rotor.name = 'rotor' + str(index)

        constructionPlane = root.xYConstructionPlane
        if (index > 0):
            planes = rotor.constructionPlanes
            planeInput = planes.createInput()
            planeInput.setByOffset(constructionPlane, adsk.core.ValueInput.createByReal(index * rotorThickness))
            constructionPlane = planes.add(planeInput)

        sk = rotor.sketches.add(constructionPlane)

        points = adsk.core.ObjectCollection.create()

        (xs, ys) = getPoint(0, d, sigma, E, N, Dp)
        points.add(adsk.core.Point3D.create(xs,ys,0))

        et = 2 * math.pi / (N-1)
        (xe, ye) = getPoint(et, d, sigma, E, N, Dp)
        x = xs
        y = ys
        dist = 0
        ct = 0
        dt = math.pi / N
        numPoints = 0

        while ((math.sqrt((x-xe)**2 + (y-ye)**2) > maxDist or ct < et/2) and ct < et): #close enough to the end to call it, but over half way
            (xt, yt) = getPoint(ct+dt, d, sigma, E, N, Dp)
            dist = getDist(x, y, xt, yt)

            ddt = dt/2
            lastTooBig = False
            lastTooSmall = False

            MaxLoops = 20
            loopCount = 0
            while (dist > maxDist or dist < minDist):
                loopCount = loopCount + 1
                if (loopCount > MaxLoops):
                    ui.messageBox('Unable to minimize error, resulting shapes may have issues')
                    break

                if (dist > maxDist):
                    if (lastTooSmall):
                        ddt /= 2

                    lastTooSmall = False
                    lastTooBig = True

                    if (ddt > dt/2):
                        ddt = dt/2

                    dt -= ddt

                elif (dist < minDist):
                    if (lastTooBig):
                        ddt /= 2

                    lastTooSmall = True
                    lastTooBig = False
                    dt += ddt


                (xt, yt) = getPoint(ct+dt, d, sigma, E, N, Dp)
                dist = getDist(x, y, xt, yt)

            x = xt
            y = yt
            points.add(adsk.core.Point3D.create(x,y,0))
            numPoints += 1
            ct += dt

        points.add(adsk.core.Point3D.create(xe,ye,0))
        crv = sk.sketchCurves.sketchFittedSplines.add(points)

        lines = sk.sketchCurves.sketchLines
        line1 = lines.addByTwoPoints(adsk.core.Point3D.create(0, 0, 0), crv.startSketchPoint)
        line2 = lines.addByTwoPoints(line1.startSketchPoint, crv.endSketchPoint)

        prof = sk.profiles.item(0)
        distance = adsk.core.ValueInput.createByReal(rotorThickness)

        # Get extrude features
        extrudes = rotor.features.extrudeFeatures
        extrude1 = extrudes.addSimple(prof, distance, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

        # Get the extrusion body
        body1 = extrude1.bodies.item(0)
        body1.name = "rotor" + str(index)

        inputEntities = adsk.core.ObjectCollection.create()
        inputEntities.add(body1)

        # Get Z axis for circular pattern
        zAxis = rotor.zConstructionAxis

        # Create the input for circular pattern
        circularFeats = rotor.features.circularPatternFeatures
        circularFeatInput = circularFeats.createInput(inputEntities, zAxis)

        # Set the quantity of the elements
        circularFeatInput.quantity = adsk.core.ValueInput.createByReal(N-1)

        # Set the angle of the circular pattern
        circularFeatInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')

        # Set symmetry of the circular pattern
        circularFeatInput.isSymmetric = True

        # Create the circular pattern
        circularFeat = circularFeats.add(circularFeatInput)

        ToolBodies = adsk.core.ObjectCollection.create()
        for b in circularFeat.bodies:
            ToolBodies.add(b)

        combineInput = rotor.features.combineFeatures.createInput(body1, ToolBodies)
        combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        combineInput.isNewComponent = False

        rotor.features.combineFeatures.add(combineInput)

        # create center hole
        centerHoleSketch = rotor.sketches.add(constructionPlane)
        sketchCircles = centerHoleSketch.sketchCurves.sketchCircles
        centerPoint = adsk.core.Point3D.create(0, 0, 0)
        sketchCircles.addByCenterRadius(centerPoint, bore/2)

        centerHoleProfile = centerHoleSketch.profiles.item(0)

        distance = adsk.core.ValueInput.createByReal(rotorThickness)
        centerExtrudes = rotor.features.extrudeFeatures.addSimple(centerHoleProfile, distance, adsk.fusion.FeatureOperations.CutFeatureOperation)


        #Create holes for pins

        if numHoles != 0:
            pinHoleSketch = rotor.sketches.add(constructionPlane)
            sketchCircles = pinHoleSketch.sketchCurves.sketchCircles
            hx = (holeCircleDiameter / 2) * math.cos(holeRotation)
            hy = (holeCircleDiameter / 2) * math.sin(holeRotation)
            centerPoint = adsk.core.Point3D.create(hx, hy, 0)
            sketchCircles.addByCenterRadius(centerPoint, holePinDiameter/2 + E)

            pinHoleProfile = pinHoleSketch.profiles.item(0)

            distance = adsk.core.ValueInput.createByReal(rotorThickness)
            pinExtrudes = rotor.features.extrudeFeatures.addSimple(pinHoleProfile, distance, adsk.fusion.FeatureOperations.CutFeatureOperation)

            inputEntites = adsk.core.ObjectCollection.create()
            inputEntites.add(pinExtrudes)

            # Get Z axis for circular pattern
            zAxis = rotor.zConstructionAxis

            # Create the input for circular pattern
            circularFeats = rotor.features.circularPatternFeatures
            circularFeatInput = circularFeats.createInput(inputEntites, zAxis)

            # Set the quantity of the elements
            circularFeatInput.quantity = adsk.core.ValueInput.createByReal(numHoles)

            # Set the angle of the circular pattern
            circularFeatInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')

            # Set symmetry of the circular pattern
            circularFeatInput.isSymmetric = True

            # Create the circular pattern
            circularFeat = circularFeats.add(circularFeatInput)

        #Offset the rotor to make the shaft rotate concentric with origin
        transform = rotorOcc.transform
        transform.translation = adsk.core.Vector3D.create(rotorOffset, 0, 0)
        rotorOcc.transform = transform
        design.snapshots.add()

        #Rotate the rotor to make it fit in the housing
        if (rotorRotation != 0):
            rotation = adsk.core.Matrix3D.create()
            rotation.setToRotation(rotorRotation, rotor.zConstructionAxis.geometry.getData()[2], rotor.originConstructionPoint.geometry)
            rotInput = rotor.features.moveFeatures.createInput(inputEntities, rotation)
            moveFeat = rotor.features.moveFeatures.add(rotInput)
        

    def buildHousing(self, app, ui, D, N, Dp, fixedPinLength):
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        root = design.rootComponent

        housingOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        housing = housingOcc.component
        housing.name = 'housing'

        #add a sketch so rotor clearance is obvious
        sketches = housing.sketches
        rotorClearanceSketch = sketches.add(root.xYConstructionPlane)
        sketchCircles = rotorClearanceSketch.sketchCurves.sketchCircles
        centerPoint = adsk.core.Point3D.create(0, 0, 0)
        sketchCircles.addByCenterRadius(centerPoint, (D / 2.0))

        #add rollers
        rollerSketch = sketches.add(root.xYConstructionPlane)
        sketchCircles = rollerSketch.sketchCurves.sketchCircles
        centerPoint = adsk.core.Point3D.create((D / 2.0), 0, 0)
        sketchCircles.addByCenterRadius(centerPoint, (Dp / 2.0))

        rollerProfile = rollerSketch.profiles.item(0)
        distance = adsk.core.ValueInput.createByReal(fixedPinLength)
        rollerExtrudes = housing.features.extrudeFeatures.addSimple(rollerProfile, distance, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

        # Get the extrusion body
        roller = rollerExtrudes.bodies.item(0)
        roller.name = "roller"

        inputEntites = adsk.core.ObjectCollection.create()
        inputEntites.add(roller)

        # Create the input for circular pattern
        circularFeats = housing.features.circularPatternFeatures
        zAxis = housing.zConstructionAxis
        circularFeatInput = circularFeats.createInput(inputEntites, zAxis)

        # Set the quantity of the elements
        circularFeatInput.quantity = adsk.core.ValueInput.createByReal(N)

        # Set the angle of the circular pattern
        circularFeatInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')

        # Set symmetry of the circular pattern
        circularFeatInput.isSymmetric = True

        # Create the circular pattern
        circularFeat = circularFeats.add(circularFeatInput)


    def build(self, app, ui):
        """ Perform the features to create the component """

        # Copy parameters into local variables for ease of use
        D = self.parameters["fixedPinRingDiameter"]
        N = self.parameters["N"]
        Dp = self.parameters["fixedPinDiameter"]
        rotorThickness = self.parameters["rotorThickness"]
        bore = self.parameters["bore"]
        numGears = self.parameters["numGears"]
        numHoles = self.parameters["numHoles"]
        holeCircleDiameter = self.parameters["holeCircleDiameter"]
        holePinDiameter = self.parameters["holePinDiameter"]
        eccentricityRatio = self.parameters["eccentricityRatio"]

        maxDist = 0.25 * (Dp / 2.0) #maximum allowed distance between points
        minDist = 0.5 * maxDist #the minimum allowed distance between points
        
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        root = design.rootComponent

        for i in range(int(numGears)):
            self.buildRotor(app, ui, i, D, N, Dp, rotorThickness, bore, numHoles, holeCircleDiameter, holePinDiameter, eccentricityRatio)

        self.buildHousing(app, ui, D, N, Dp, (rotorThickness * numGears))


def getPoint(t, d, sigma, E, N, Dp):
    """ Get a point on a cycloid with the given parameters

        t: parameter
        d: cycloid base circle dia
        sigma: rolling circle dia
        E: eccentricity
        N: number of pins
        Dp: fixed pin diameter """

    h = ((d / 2.0) + (sigma / 2.0))
    p1x = h * math.cos(t)
    p1y = h * math.sin(t)
    p2x = E * math.cos(t * N)
    p2y = E * math.sin(t * N)
    px = p1x + p2x
    py = p1y + p2y
    dxdt = (-h * math.sin(t)) - (N * E * math.sin(t * N))
    dydt = (h * math.cos(t)) + (N * E * math.cos(t * N))
    denom = math.sqrt((dxdt * dxdt) + (dydt * dydt))
    dx = -((Dp / 2.0) * dydt) / denom
    dy = ((Dp / 2.0) * dxdt) / denom
    x = px + dx
    y = py + dy

    return (x,y)
 

def getDist(xa, ya, xb, yb):
    """ Get distance between two 2D points (xa,ya) and (xb,yb)"""
    return math.sqrt((xa-xb)**2 + (ya-yb)**2)
