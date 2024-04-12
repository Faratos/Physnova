import pygame as pg
import math
import sys

pg.init()

print(f"physnova 1.0 (using pygame-ce 2.4.1)")

def error(text: str):
    print("Произошла ошибка!")
    print(text)
    pg.quit()
    sys.exit()


class Transform:
    def __init__(self, position: pg.Vector2, angle: float):
        self.Position = pg.Vector2(position)
        self.Sin = math.sin(angle)
        self.Cos = math.cos(angle)


def transformVector(vector: pg.Vector2, transform: Transform):
    return pg.Vector2(transform.Cos * vector.x - transform.Sin * vector.y + transform.Position.x,
                      transform.Sin * vector.x + transform.Cos * vector.y + transform.Position.y)


class AABB:
    def __init__(self, min: pg.Vector2, max: pg.Vector2):
        self.min = pg.Vector2(min)
        self.max = pg.Vector2(max)

        self.width = max.x - min.x
        self.height = max.y - min.y


class Material:
    def __init__(self, density: float, restitution: float, staticFriction: float, dynamicFriction: float):
        self.Density = density
        self.Restitution = pg.math.clamp(restitution, 0, 1)
        self.StaticFriction = pg.math.clamp(staticFriction, 0, 1)
        self.DynamicFriction = pg.math.clamp(dynamicFriction, 0, 1)


class Rigidbody:
    def __init__(self, mass: float, area: float, material: Material, isStatic: bool, radius: float, width: float, height: float, shape: str):
        self.Shape = shape
        self.Surface = None
        self.__transformedSurface = None
        if self.Shape == "box":
            self.Surface = pg.Surface((width, height)).convert_alpha()
            self.Surface.fill("black")
        if self.Shape == "circle":
            self.Surface = pg.Surface((radius * 2, radius * 2)).convert_alpha()
            self.Surface.fill((0, 0, 0, 0))
            pg.draw.circle(self.Surface, "black", (radius, radius), radius)

        self.Radius = radius
        self.Width = width
        self.Height = height

        self.Mass = mass
        self.InvMass = 0
        self.Area = area
        self.Inertia = 0
        self.InvInertia = 0
        self.Material = material

        self.IsStatic = isStatic
        
        self.Position = pg.Vector2()
        self.LinearVelocity = pg.Vector2()
        self.Angle = 0
        self.AngularVelocity = 0

        self.Force = pg.Vector2()

        self.__transformUpdateRequired = True
        self.__aabbUpdateRequired = True
        self.__surfaceUpdateRequired = True

        self.Vertices: list[pg.Vector2] = []
        self.__transformedVertices: list[pg.Vector2] = []
        self.__aabb: AABB = None

        if shape == "box":
            self.Vertices = self.__CreateBoxVertices(width, height)
            self.__transformedVertices = [pg.Vector2() for _ in range(len(self.Vertices))]

    def Update(self, dt: float, gravity: pg.Vector2):
        if not self.IsStatic:
            self.LinearVelocity += gravity * dt

        acceleration = self.Force / self.Mass

        self.LinearVelocity += acceleration * dt

        self.Position += self.LinearVelocity * dt
        self.Angle += self.AngularVelocity * dt

        self.Force.xy = (0, 0)

        self.__transformUpdateRequired = True
        self.__aabbUpdateRequired = True
        self.__surfaceUpdateRequired = True

    def __CreateBoxVertices(self, width: int, height: int):
        left =  -width / 2
        right = left + width
        top = -height / 2
        bottom = top + height
        
        return [pg.Vector2(left, top),
                pg.Vector2(right, top),
                pg.Vector2(right, bottom),
                pg.Vector2(left, bottom)]
    
    def Move(self, value: pg.Vector2):
        if value.xy != (0, 0):
            self.Position += value
            self.__transformUpdateRequired = True
            self.__aabbUpdateRequired = True

    def SetPosition(self, value: pg.Vector2 | tuple):
        if value.xy != self.Position.xy:
            self.Position = pg.Vector2(value)
            self.__transformUpdateRequired = True
            self.__aabbUpdateRequired = True

    def ApplyForce(self, value: pg.Vector2):
        self.Force += value

    def ApplyAngularForce(self, value: float):
        self.AngularVelocity += value

    def Rotate(self, value: int):
        if value != 0:
            self.Angle += math.radians(value)
            self.__transformUpdateRequired = True
            self.__aabbUpdateRequired = True
            self.__surfaceUpdateRequired = True

    def SetAngle(self, value: int):
        radiansValue = math.radians(value)
        if radiansValue != self.Angle:
            self.Angle = radiansValue
            self.__transformUpdateRequired = True
            self.__aabbUpdateRequired = True
            self.__surfaceUpdateRequired = True

    def GetAABB(self):
        if self.__aabbUpdateRequired:
            min = pg.Vector2(1e+20, 1e+20)
            max = pg.Vector2(-1e+20, -1e+20)

            if self.Shape == "box":
                vertices = self.GetTransformedVertices()

                for i in range(len(vertices)):
                    vert = vertices[i]

                    if vert.x < min.x: min.x = vert.x
                    if vert.x > max.x: max.x = vert.x
                    if vert.y < min.y: min.y = vert.y
                    if vert.y > max.y: max.y = vert.y

            if self.Shape == "circle":
                min.x = self.Position.x - self.Radius
                min.y = self.Position.y - self.Radius

                max.x = self.Position.x + self.Radius
                max.y = self.Position.y + self.Radius

            self.__aabb = AABB(min, max)

            self.__aabbUpdateRequired = False
        
        return self.__aabb

    def GetTransformedVertices(self):
        if self.__transformUpdateRequired:
            transform = Transform(self.Position, self.Angle)

            for i in range(len(self.Vertices)):
                vector = self.Vertices[i]
                self.__transformedVertices[i] = transformVector(vector, transform)

            self.__transformUpdateRequired = False
        
        return self.__transformedVertices

    def GetSurface(self):
        if not self.__surfaceUpdateRequired:
            return self.__transformedSurface
        
        self.__transformedSurface = pg.transform.rotate(self.Surface, math.degrees(-self.Angle))
        self.__surfaceUpdateRequired = False
        return self.__transformedSurface

    def Draw(self, screen: pg.Surface, cameraZoom: int, cameraPosition: pg.Vector2, showHitbox: bool=False):
        camPos = cameraPosition - pg.Vector2(screen.get_width() / 2, screen.get_height() / 2)
        surface = self.GetSurface()
        position = self.Position * cameraZoom - camPos
        if self.Shape == "circle":
            radius = self.Radius * cameraZoom
            
            screen.blit(surface, position - pg.Vector2(surface.get_size()) / 2)
            
            if showHitbox:
                pg.draw.circle(screen, "red", position, radius, 2)
                angleLine = pg.Vector2(radius, 0)
                angleLine.rotate_ip(math.degrees(self.Angle))
                pg.draw.line(screen, "red", position, position + angleLine, 2)

        if self.Shape == "box":
            screen.blit(surface, position - pg.Vector2(surface.get_size()) / 2)
            if showHitbox:
                vertices = [i * cameraZoom - camPos for i in self.GetTransformedVertices()]
                pg.draw.polygon(screen, "white", vertices, 2)

    def SetSurface(self, surface: pg.Surface):
        if self.Shape == "box":
            self.Surface = pg.transform.scale(surface, (self.Width, self.Height)).convert_alpha()

        if self.Shape == "circle":
            scaledSurface = pg.transform.scale(surface, (self.Radius * 2, self.Radius * 2))
            self.Surface.fill((0, 0, 0, 0))
            pg.draw.circle(self.Surface, "black", (self.Radius, self.Radius), self.Radius)
            self.Surface.blit(scaledSurface, (0, 0), special_flags=pg.BLEND_RGB_ADD)

    def CreateSurface(self, mainColor: tuple, outlineColor: tuple, outlineWidth: int):
        if self.Shape == "box":
            self.Surface.fill(mainColor)
            pg.draw.rect(self.Surface, outlineColor, (0, 0, self.Width, self.Height), outlineWidth)

        if self.Shape == "circle":
            pg.draw.circle(self.Surface, mainColor, (self.Radius, self.Radius), self.Radius)
            pg.draw.circle(self.Surface, outlineColor, (self.Radius, self.Radius), self.Radius, outlineWidth)

    def __str__(self):
        return f"Rigidbody(Pos: {self.Position}, Angle: {self.Angle}, Size:[{self.Width}, {self.Height}])"
    
    def __repr__(self):
        return self.__str__()


def nearlyEqual(a: float, b: float):
        return abs(a - b) < 0.001

def vectorsNearlyEqual(a: pg.Vector2, b: pg.Vector2):
    return a.distance_squared_to(b) < 0.005**2


class Collisions:
    @staticmethod
    def __ProjectVertices(vertices: list[pg.Vector2], axis: pg.Vector2):
        min = 1.7976931348623157e+308
        max = -1.7976931348623157e+308

        for i in range(len(vertices)):
            vert = vertices[i]
            projection = vert.dot(axis)

            if projection < min:
                min = projection

            if projection > max:
                max = projection

        return min, max
    
    @staticmethod
    def __ProjectCircle(center: pg.Vector2, radius: float, axis: pg.Vector2):
        direction = axis.normalize()
        directionAndRadius = direction * radius

        point1 = center + directionAndRadius
        point2 = center - directionAndRadius

        min = point1.dot(axis)
        max = point2.dot(axis)

        if min > max:
            min, max = max, min

        return min, max

    @staticmethod
    def __ClosestPointOnPolygon(circleCenter: pg.Vector2, vertices: list[pg.Vector2]):
        result = 0
        minDist = 1.7976931348623157e+308

        for i in range(len(vertices)):
            vector = vertices[i]
            distance = vector.distance_squared_to(circleCenter)
            if (distance < minDist):
                minDist = distance
                result = i

        return result

    @staticmethod
    def __PointSegmentDistance(point: pg.Vector2, a: pg.Vector2, b: pg.Vector2):
        ab = b - a
        ap = point - a

        proj = ap.dot(ab)
        abLenSqr = ab.length_squared()
        d = proj / abLenSqr

        if d <= 0: closestPoint = a
        elif d >= 1: closestPoint = b
        else: closestPoint = a + ab * d

        distanceSquared = point.distance_squared_to(closestPoint)

        return closestPoint, distanceSquared

    @staticmethod
    def FindContactPoints(first: Rigidbody, second: Rigidbody):
        if first.Shape == "circle" and second.Shape == "circle":
            return first.Position + (second.Position - first.Position).normalize() * first.Radius, None, 1
        
        if first.Shape == "box" and second.Shape == "box":
            contact1 = pg.Vector2()
            contact2 = pg.Vector2()
            contactCount = 0

            verticesA = first.GetTransformedVertices()
            verticesB = second.GetTransformedVertices()

            minDist = 1.0e+10

            for i in range(len(verticesA)):
                point = verticesA[i]

                for j in range(len(verticesB)):
                    vertA = verticesB[j]
                    vertB = verticesB[(j + 1) % len(verticesB)]

                    closestPoint, distance = Collisions.__PointSegmentDistance(point, vertA, vertB)

                    if nearlyEqual(distance, minDist) and not vectorsNearlyEqual(closestPoint, contact1):
                        contact2 = closestPoint
                        contactCount = 2

                    if distance < minDist:
                        minDist = distance
                        contactCount = 1
                        contact1 = closestPoint

            for i in range(len(verticesB)):
                point = verticesB[i]

                for j in range(len(verticesA)):
                    vertA = verticesA[j]
                    vertB = verticesA[(j + 1) % len(verticesA)]

                    closestPoint, distance = Collisions.__PointSegmentDistance(point, vertA, vertB)

                    if nearlyEqual(distance, minDist) and not vectorsNearlyEqual(closestPoint, contact1):
                        contact2 = closestPoint
                        contactCount = 2

                    if distance < minDist:
                        minDist = distance
                        contactCount = 1
                        contact1 = closestPoint

            return contact1, contact2, contactCount
        
        if (first.Shape == "circle" and second.Shape == "box") or (second.Shape == "circle" and first.Shape == "box"):
            circle, box = (first, second) if first.Shape == "circle" else (second, first)
            vertices = box.GetTransformedVertices()

            point = pg.Vector2()

            minDist = 1.0e+10
            for i in range(len(vertices)):
                vertA = vertices[i]
                vertB = vertices[(i + 1) % len(vertices)]

                contact, distance = Collisions.__PointSegmentDistance(circle.Position, vertA, vertB)

                if distance < minDist:
                    minDist = distance
                    point = contact

            return point, None, 1
        
        return None, None, 0

    @staticmethod
    def IntersectCircles(firstPosition: pg.Vector2, firstRadius: float, secondPosition: pg.Vector2, secondRadius: int):
        isCollided, normal, depth = False, pg.Vector2(), 0

        distance = firstPosition.distance_to(secondPosition)
        radius = firstRadius + secondRadius
        if distance >= radius:
            return False, pg.Vector2(), 0
        
        normal = (secondPosition - firstPosition)
        normal.xy = (0, 1) if normal.xy == (0, 0) else normal.normalize()
        depth = radius - distance
        isCollided = True

        return isCollided, normal, depth

    @staticmethod
    def IntersectPolygons(verticesA: list[pg.Vector2], positionA: pg.Vector2, verticesB: list[pg.Vector2], positionB: pg.Vector2):
        isCollided, normal, depth = False, pg.Vector2(), 0

        depth = 1.7976931348623157e+308
        for i in range(len(verticesA)):
            vertA = verticesA[i]
            vertB = verticesA[(i + 1) % len(verticesA)]

            edge = vertB - vertA
            axis = pg.Vector2(-edge.y, edge.x)
            axis.normalize_ip()

            minA, maxA = Collisions.__ProjectVertices(verticesA, axis)
            minB, maxB = Collisions.__ProjectVertices(verticesB, axis)

            if minA >= maxB or minB >= maxA:
                return False, pg.Vector2(), 0
            
            axisDepth = min(maxB - minA, maxA - minB)

            if axisDepth < depth:
                depth = axisDepth
                normal = axis
            
        for i in range(len(verticesB)):
            vertA = verticesB[i]
            vertB = verticesB[(i + 1) % len(verticesB)]

            edge = vertB - vertA
            axis = pg.Vector2(-edge.y, edge.x)
            axis.normalize_ip()

            minA, maxA = Collisions.__ProjectVertices(verticesA, axis)
            minB, maxB = Collisions.__ProjectVertices(verticesB, axis)

            if minA >= maxB or minB >= maxA:
                return False, pg.Vector2(), 0
            
            axisDepth = min(maxB - minA, maxA - minB)

            if axisDepth < depth:
                depth = axisDepth
                normal = axis
            
        isCollided = True

        direction = positionB - positionA

        if (direction.dot(normal) < 0):
            normal = -normal

        return isCollided, normal, depth

    @staticmethod
    def IntersectPolygonCircle(vertices: list[pg.Vector2], polygonPosition: pg.Vector2, radius: float, circlePosition: pg.Vector2, isFirstABox: bool):
        isCollided, normal, depth = False, pg.Vector2(), 0

        depth = 1.7976931348623157e+308

        axis = pg.Vector2()

        for i in range(len(vertices)):
            vertA = vertices[i]
            vertB = vertices[(i + 1) % len(vertices)]

            edge = vertB - vertA
            axis = pg.Vector2(-edge.y, edge.x)
            axis.normalize_ip()

            minA, maxA = Collisions.__ProjectVertices(vertices, axis)
            minB, maxB = Collisions.__ProjectCircle(circlePosition, radius, axis)

            if minA >= maxB or minB >= maxA:
                return False, pg.Vector2(), 0
            
            axisDepth = min(maxB - minA, maxA - minB)

            if axisDepth < depth:
                depth = axisDepth
                normal = axis

        point = vertices[Collisions.__ClosestPointOnPolygon(circlePosition, vertices)]

        axis = point - circlePosition
        axis.normalize_ip()

        minA, maxA = Collisions.__ProjectVertices(vertices, axis)
        minB, maxB = Collisions.__ProjectCircle(circlePosition, radius, axis)

        if minA >= maxB or minB >= maxA:
            return False, pg.Vector2(), 0
        
        axisDepth = min(maxB - minA, maxA - minB)

        if axisDepth < depth:
            depth = axisDepth
            normal = axis

        isCollided = True

        direction = polygonPosition - circlePosition

        if (isFirstABox and direction.dot(normal) > 0) or (not isFirstABox and direction.dot(normal) < 0):
            normal = -normal
    
        return isCollided, normal, depth

    @staticmethod
    def CollideBodies(first: Rigidbody, second: Rigidbody):
        isCollided, normal, depth = False, pg.Vector2(), 0
        
        if (first.Shape == "circle" and second.Shape == "circle"):
            isCollided, normal, depth = Collisions.IntersectCircles(first.Position, first.Radius, second.Position, second.Radius)
        
        if (first.Shape == "box" and second.Shape == "box"):
            isCollided, normal, depth = Collisions.IntersectPolygons(first.GetTransformedVertices(), first.Position, second.GetTransformedVertices(), second.Position)
            
        if ((first.Shape == "box" and second.Shape == "circle") or (second.Shape == "box" and first.Shape == "circle")):
            polygon = second if second.Shape == "box" else first
            circle = first if first.Shape == "circle" else second

            isCollided, normal, depth = Collisions.IntersectPolygonCircle(polygon.GetTransformedVertices(), polygon.Position, circle.Radius, circle.Position, first.Shape == "box")

        return isCollided, normal, depth

    @staticmethod
    def CollideAABB(first: AABB, second: AABB):
        if (first.max.x <= second.min.x or second.max.x <= first.min.x or
            first.max.y <= second.min.y or second.max.y <= first.min.y):
            return False
        
        return True

    @staticmethod
    def CollidePoint(rigidbody: Rigidbody, point: pg.Vector2):
        if rigidbody.Shape == "circle":
            return rigidbody.Position.distance_squared_to(point) < rigidbody.Radius**2
        
        elif rigidbody.Shape == "box":
            vertices = rigidbody.GetTransformedVertices()
            
            collision = False

            next = 0
            for current in range(len(vertices)):

                next = current + 1
                if next >= len(vertices): 
                    next = 0

                currentPoint = vertices[current]
                nextPoint = vertices[next]

                if (((currentPoint.y >= point.y and nextPoint.y < point.y) or (currentPoint.y < point.y and nextPoint.y >= point.y)) and 
                        (point.x < (nextPoint.x - currentPoint.x) * (point.y - currentPoint.y) / (nextPoint.y - currentPoint.y) + currentPoint.x)):
                    collision = not collision
            
            return collision
    
        return False


class CollisionManifold:
    def __init__(self, bodyA: Rigidbody, bodyB: Rigidbody, normal: pg.Vector2, depth: int, contacts: tuple):
        self.BodyA = bodyA
        self.BodyB = bodyB
        self.Normal = normal
        self.Depth = depth
        self.Contact1 = contacts[0]
        self.Contact2 = contacts[1]
        self.ContactCount = contacts[2]


class Resolver:
    @staticmethod
    def __ResolveCollisionsBasic(contact: CollisionManifold):
        first = contact.BodyA
        second = contact.BodyB
        normal = contact.Normal

        relativeVelocity = second.LinearVelocity - first.LinearVelocity

        if relativeVelocity.dot(normal) > 0:
            return

        e = min(first.Material.Restitution, second.Material.Restitution)
        
        j = (-(1 + e) * relativeVelocity.dot(normal)) / (first.InvMass + second.InvMass)

        impulse = j * normal

        first.LinearVelocity -= impulse * first.InvMass
        second.LinearVelocity += impulse * second.InvMass

    @staticmethod
    def __ResolveCollisionsWithRotation(contact: CollisionManifold):
        first = contact.BodyA
        second = contact.BodyB
        normal = contact.Normal

        contact1: pg.Vector2 = contact.Contact1
        contact2: pg.Vector2 = contact.Contact2
        contactCount = contact.ContactCount

        e = min(first.Material.Restitution, second.Material.Restitution)

        contactList = [contact1, contact2]
        impulses: list[pg.Vector2] = []
        raList: list[pg.Vector2] = []
        rbList: list[pg.Vector2] = []

        for i in range(contactCount):
            ra = contactList[i] - first.Position
            rb = contactList[i] - second.Position

            raList.append(ra)
            rbList.append(rb)
            
            raPerp = pg.Vector2(-ra.y, ra.x)
            rbPerp = pg.Vector2(-rb.y, rb.x)

            angularLinearVelocityA = raPerp * first.AngularVelocity
            angularLinearVelocityB = rbPerp * second.AngularVelocity

            relativeVelocity = ((second.LinearVelocity + angularLinearVelocityB) - 
                                (first.LinearVelocity + angularLinearVelocityA))

            contactVelocityMag = relativeVelocity.dot(normal)

            if contactVelocityMag > 0:
                continue

            raPerpDotN = raPerp.dot(normal)
            rbPerpDotN = rbPerp.dot(normal)

            denom = (first.InvMass + second.InvMass + 
                        raPerpDotN**2 * first.InvInertia + 
                        rbPerpDotN**2 * second.InvInertia)

            j = -(1 + e) * contactVelocityMag
            j /= denom
            j /= contactCount

            impulse = j * normal
            impulses.append(impulse)

        for i in range(len(impulses)):
            impulse = impulses[i]

            first.LinearVelocity += -impulse * first.InvMass
            first.AngularVelocity += -raList[i].cross(impulse) * first.InvInertia

            second.LinearVelocity += impulse * second.InvMass
            second.AngularVelocity += rbList[i].cross(impulse) * second.InvInertia

    @staticmethod
    def __ResolveCollisionsWithRotationAndFriction(contact: CollisionManifold):
        first = contact.BodyA
        second = contact.BodyB
        normal = contact.Normal

        contact1: pg.Vector2 = contact.Contact1
        contact2: pg.Vector2 = contact.Contact2
        contactCount = contact.ContactCount

        e = min(first.Material.Restitution, second.Material.Restitution)

        staticFriction = (first.Material.StaticFriction + second.Material.StaticFriction) / 2
        dynamicFriction = (first.Material.DynamicFriction + second.Material.DynamicFriction) / 2

        contactList = [contact1, contact2]
        impulses: list[pg.Vector2] = []
        raList: list[pg.Vector2] = []
        rbList: list[pg.Vector2] = []
        jList: list[int] = []

        for i in range(contactCount):
            ra = contactList[i] - first.Position
            rb = contactList[i] - second.Position

            raList.append(ra)
            rbList.append(rb)
            
            raPerp = pg.Vector2(-ra.y, ra.x)
            rbPerp = pg.Vector2(-rb.y, rb.x)

            angularLinearVelocityA = raPerp * first.AngularVelocity
            angularLinearVelocityB = rbPerp * second.AngularVelocity

            relativeVelocity = ((second.LinearVelocity + angularLinearVelocityB) - 
                                (first.LinearVelocity + angularLinearVelocityA))

            contactVelocityMag = relativeVelocity.dot(normal)

            if contactVelocityMag > 0:
                continue

            raPerpDotN = raPerp.dot(normal)
            rbPerpDotN = rbPerp.dot(normal)

            denom = (first.InvMass + second.InvMass + 
                        raPerpDotN**2 * first.InvInertia + 
                        rbPerpDotN**2 * second.InvInertia)

            j = -(1 + e) * contactVelocityMag
            j /= denom
            j /= contactCount

            jList.append(j)

            impulse = j * normal
            impulses.append(impulse)

        for i in range(len(impulses)):
            impulse = impulses[i]

            first.LinearVelocity += -impulse * first.InvMass
            first.AngularVelocity += -raList[i].cross(impulse) * first.InvInertia

            second.LinearVelocity += impulse * second.InvMass
            second.AngularVelocity += rbList[i].cross(impulse) * second.InvInertia

        frictionImpulses = []

        for i in range(contactCount):
            ra = contactList[i] - first.Position
            rb = contactList[i] - second.Position

            raList[i] = ra
            rbList[i] = rb
            
            raPerp = pg.Vector2(-ra.y, ra.x)
            rbPerp = pg.Vector2(-rb.y, rb.x)

            angularLinearVelocityA = raPerp * first.AngularVelocity
            angularLinearVelocityB = rbPerp * second.AngularVelocity

            relativeVelocity = ((second.LinearVelocity + angularLinearVelocityB) - 
                                (first.LinearVelocity + angularLinearVelocityA))

            tangent = relativeVelocity - relativeVelocity.dot(normal) * normal

            if (vectorsNearlyEqual(tangent, pg.Vector2())):
                continue

            tangent.normalize_ip()

            raPerpDotT = raPerp.dot(tangent)
            rbPerpDotT = rbPerp.dot(tangent)

            denom = (first.InvMass + second.InvMass + 
                        raPerpDotT**2 * first.InvInertia + 
                        rbPerpDotT**2 * second.InvInertia)

            jt = -relativeVelocity.dot(tangent)
            jt /= denom
            jt /= contactCount

            impulse = pg.Vector2()

            if i < len(jList):
                if abs(jt) <= jList[i] * staticFriction:
                    impulse = jt * tangent
                else:
                    impulse = -jList[i] * tangent * dynamicFriction

                frictionImpulses.append(impulse)

        for i in range(len(frictionImpulses)):
            impulse = frictionImpulses[i]

            first.LinearVelocity += -impulse * first.InvMass
            first.AngularVelocity += -raList[i].cross(impulse) * first.InvInertia

            second.LinearVelocity += impulse * second.InvMass
            second.AngularVelocity += rbList[i].cross(impulse) * second.InvInertia

    resolvingCollisionsFunctions = [__ResolveCollisionsBasic, __ResolveCollisionsWithRotation, __ResolveCollisionsWithRotationAndFriction]

    @staticmethod
    def SeparateBodies(first: Rigidbody, second: Rigidbody, mtv: pg.Vector2):
        if first.IsStatic:
            second.Move(mtv)
        elif second.IsStatic:
            first.Move(-mtv)
        else:
            first.Move(-mtv / 2)
            second.Move(mtv / 2)

    @staticmethod
    def BroadPhase(bodies: list[Rigidbody], contactPairs: list[tuple[int, int]]):
        for i in range(len(bodies)):
            mainBody = bodies[i]
            mainAABB = mainBody.GetAABB()

            if i < len(bodies) - 1:
                for j in range(i + 1, len(bodies)):
                    otherBody = bodies[j]
                    otherAABB = otherBody.GetAABB()

                    if (mainBody.IsStatic and otherBody.IsStatic) or not Collisions.CollideAABB(mainAABB, otherAABB):
                        continue

                    contactPairs.append((i, j))

    @staticmethod   
    def NarrowPhase(bodies: list[Rigidbody], contactPairs: list[tuple[int, int]], resolvingCollisionMethod: int = 2):
        for pair in contactPairs:
            mainBody = bodies[pair[0]]
            otherBody = bodies[pair[1]]

            isCollided, normal, depth = Collisions.CollideBodies(mainBody, otherBody)
                
            if isCollided:
                Resolver.SeparateBodies(mainBody, otherBody, normal * depth)
                resolvingCollisionMethod = pg.math.clamp(resolvingCollisionMethod, 0, 2)
                Resolver.resolvingCollisionsFunctions[resolvingCollisionMethod](CollisionManifold(mainBody, otherBody, normal, depth, Collisions.FindContactPoints(mainBody, otherBody)))

    @staticmethod
    def UpdateBodies(dt: float, gravity: pg.Vector2, bodies: list[Rigidbody]):
        for body in bodies:
            body.Update(dt, gravity)


class World:
    def __init__(self):
        self.MinBodySize = 0.01**2 * 30
        self.MaxBodySize = 64**2 * 30

        self.MinDensity = 0.5
        self.MaxDensity = 23

        self.MinIterations = 1
        self.MaxIterations = 128

        self.__gravity = pg.Vector2(0, 9.81 * 37.8)

        self.__bodies: list[Rigidbody] = []

        self.__contactPairs: list[tuple[int, int]] = []

        self.BodyCount = 0

        self.CameraZoom = 1
        self.CameraPosition = pg.Vector2()

    def CameraLookAtPoint(self, point: pg.Vector2, smooth: float):
        smooth = pg.math.clamp(smooth, 1, 100)
        self.CameraPosition += (point - self.CameraPosition) / smooth
    
    def ZoomCamera(self, value: int):
        self.CameraZoom += value

    def MoveCamera(self, value: pg.Vector2):
        self.CameraPosition += value

    def CreateCircleBody(self, material: Material, isStatic: bool, radius: float, freezeRotation=False):
        area = radius ** 2 * math.pi
        if area < self.MinBodySize:
            error(f"Радиус окружности слишком маленький. Минимальная площадь - {self.MinBodySize}.")
        
        if area > self.MaxBodySize:
            error(f"Радиус окружности слишком большой. Максимальная площадь - {self.MaxBodySize}.")
        
        if material.Density < self.MinDensity:
            error(f"Плотность материала слишком маленькая. Минимальная плотность - {self.MinDensity}.")
        
        if material.Density > self.MaxDensity:
            error(f"Плотность материала слишком большая. Максимальная плотность - {self.MaxDensity}.")
        
        mass = area * material.Density

        body = Rigidbody(mass, area, material, isStatic, radius, 0, 0, "circle")
        body.Inertia = (1 / 2) * mass * radius**2

        if not body.IsStatic:
            if not freezeRotation:
                body.InvInertia = 1 / body.Inertia
            body.InvMass = 1 / body.Mass
            
        self.__bodies.append(body)

        return body
    
    def CreateBoxBody(self, material: Material, isStatic: bool, width: float, height: float, freezeRotation=False):
        area = width * height
        if area < self.MinBodySize:
            error(f"Ширина и высота слишком маленькие. Минимальная площадь - {self.MinBodySize}.")
        
        if area > self.MaxBodySize:
            error(f"Ширина и высота слишком большие. Максимальная площадь - {self.MaxBodySize}.")
        
        if material.Density < self.MinDensity:
            error(f"Плотность тела слишком маленькая. Минимальная плотность - {self.MinDensity}.")
        
        if material.Density > self.MaxDensity:
            error(f"Плотность тела слишком маленькая. Максимальная плотность - {self.MaxDensity}.")
        
        mass = area * material.Density

        body = Rigidbody(mass, area, material, isStatic, 0, width, height, "box")
        body.Inertia = (1 / 12) * mass * (width**2 + height**2)

        if not body.IsStatic:
            if not freezeRotation:
                body.InvInertia = 1 / body.Inertia
            body.InvMass = 1 / body.Mass
        
        self.__bodies.append(body)

        return body

    def RemoveBody(self, body: Rigidbody):
        if body in self.__bodies:
            self.__bodies.remove(body)

    def RemoveBodyByIndex(self, index: int):
        if index >= 0 and index < len(self.__bodies):
            self.__bodies.pop(index)
    
    def GetRigidbody(self, index: int):
        if index >= 0 and index < len(self.__bodies):
            return self.__bodies[index]
        
        error(f"Тела с индексом \"{index}\" не существует.")
    
    def Update(self, screen: pg.Surface, dt: float, iterations: int):
        iterations = pg.math.clamp(iterations, self.MinIterations, self.MaxIterations)
        dt /= iterations
        for _ in range(iterations):
            self.__contactPairs.clear()

            Resolver.UpdateBodies(dt, self.__gravity, self.__bodies)
            Resolver.BroadPhase(self.__bodies, self.__contactPairs)
            Resolver.NarrowPhase(self.__bodies, self.__contactPairs, 2)

        self.BodyCount = len(self.__bodies)

        for i in range(len(self.__bodies)):
            self.__bodies[i].Draw(screen, self.CameraZoom, self.CameraPosition)