import pygame as pg
import math
import sys

pg.init()

"""

   -={ (beta) PhysNova 1.1 }=-
   -={      By Faratos     }=-

"""

def error(text: str):
    print("Произошла ошибка!")
    print(text)
    pg.quit()
    sys.exit()


class Transform:
    def __init__(self, position: pg.Vector2, angle: float):
        self.position = pg.Vector2(position)
        self.sin = math.sin(angle)
        self.cos = math.cos(angle)


def TransformVector(vector: pg.Vector2, transform: Transform):
    return pg.Vector2(transform.cos * vector.x - transform.sin * vector.y + transform.position.x,
                      transform.sin * vector.x + transform.cos * vector.y + transform.position.y)

class AABB:
    def __init__(self, min: pg.Vector2, max: pg.Vector2):
        self.min = pg.Vector2(min)
        self.max = pg.Vector2(max)

        self.width = max.x - min.x
        self.height = max.y - min.y


class Material:
    def __init__(self, density: float, restitution: float, staticFriction: float, dynamicFriction: float, airFriction: float = 0.002):
        self.density = density
        self.restitution = pg.math.clamp(restitution, 0, 1)
        self.staticFriction = pg.math.clamp(staticFriction, 0, 1)
        self.dynamicFriction = pg.math.clamp(dynamicFriction, 0, 1)
        self.airFriction = pg.math.clamp(airFriction, 0, 1)

    def Copy(self):
        return Material(self.density, self.restitution, self.staticFriction, self.dynamicFriction)


class Rigidbody:
    def __init__(self, mass: float, area: float, material: Material, isStatic: bool, radius: float, width: float, height: float, shape: str):
        self.shape = shape
        self.surface = None
        self.__transformedSurface = None
        if self.shape == "box":
            self.surface = pg.Surface((width, height)).convert_alpha()
            self.surface.fill("black")
        if self.shape == "circle":
            self.surface = pg.Surface((radius * 2, radius * 2)).convert_alpha()
            self.surface.fill((0, 0, 0, 0))
            pg.draw.circle(self.surface, "black", (radius, radius), radius)
        if self.shape == "capsule":
            self.surface = pg.Surface((radius * 2, radius * 2 + height)).convert_alpha()
            self.surface.fill((0, 0, 0, 0))
            pg.draw.circle(self.surface, "black", (radius, radius), radius)
            pg.draw.circle(self.surface, "black", (radius, radius + height), radius)
            pg.draw.rect(self.surface, "black", (0, radius, radius * 2, height))

        self.radius = radius
        self.width = width
        self.height = height

        self.mass = mass
        self.invMass = 0
        self.area = area
        self.inertia = 0
        self.invInertia = 0
        self.material = material

        self.isStatic = isStatic
        
        self.position = pg.Vector2()
        self.linearVelocity = pg.Vector2()
        self.angle = 0
        self.angularVelocity = 0

        self.force = pg.Vector2()
        self.torque = 0

        self.__transformUpdateRequired = True
        self.__aabbUpdateRequired = True
        self.__surfaceUpdateRequired = True
        self.__anchorsUpdateRequired = True

        self.vertices: list[pg.Vector2] = []
        self.__aabb: AABB = None

        self.anchors: list[pg.Vector2] = []
        self.__transformedAnchors: list[pg.Vector2] = []

        self.cannotCollideWith: list[Rigidbody] = []
        
        self.visible = True

        if shape == "box":
            self.vertices = self.__createBoxVertices(width, height)

        if shape == "capsule":
            self.vertices = self.__createCapsuleVertices(height)

        self.__transformedVertices: list[pg.Vector2] = [pg.Vector2() for _ in range(len(self.vertices))]

    def Update(self, dt: float, gravity: pg.Vector2):
        if not self.isStatic:
            self.linearVelocity += gravity * dt

        acceleration = self.force / self.mass

        self.linearVelocity += acceleration * dt
        self.angularVelocity += self.torque / self.inertia * dt
        
        self.linearVelocity *= 1 - self.material.airFriction
        self.angularVelocity *= 1 - self.material.airFriction

        self.position += self.linearVelocity * dt
        self.angle += self.angularVelocity * dt

        self.force.xy = (0, 0)
        self.torque = 0

        self.__transformUpdateRequired = True
        self.__anchorsUpdateRequired = True
        self.__aabbUpdateRequired = True
        self.__surfaceUpdateRequired = True

    def __createBoxVertices(self, width: int, height: int):
        left =  -width / 2
        right = left + width
        top = -height / 2
        bottom = top + height
        
        return [pg.Vector2(left, top),
                pg.Vector2(right, top),
                pg.Vector2(right, bottom),
                pg.Vector2(left, bottom)]
    
    def __createCapsuleVertices(self, height: int):
        top = -height / 2
        bottom = top + height
        
        return [pg.Vector2(0, top),
                pg.Vector2(0, bottom)]
    
    def Move(self, value: pg.Vector2):
        if not (value[0] == 0 and value[1] == 0):
            self.position += value
            self.__transformUpdateRequired = True
            self.__aabbUpdateRequired = True

    def SetPosition(self, value: pg.Vector2 | tuple):
        if not (value[0] == self.position[0] and value[1] == self.position[1]):
            self.position = pg.Vector2(value[0], value[1])
            self.__transformUpdateRequired = True
            self.__aabbUpdateRequired = True

    def ApplyForce(self, value: pg.Vector2):
        self.force += value

    def ApplyForceAtPoint(self, value: pg.Vector2, point: pg.Vector2, torqueFactor: float = 1):
        self.ApplyForceAtLocalPoint(value, point - self.position, torqueFactor)

    def ApplyForceAtLocalPoint(self, value: pg.Vector2, point: pg.Vector2, torqueFactor: float = 1):
        if point != None and value != None:
            self.force += value
            self.torque += point.cross(value) * torqueFactor

    def ApplyForceAtAnchor(self, value: pg.Vector2, anchorIndex: int, torqueFactor: float = 1):
        self.ApplyForceAtPoint(value, self.GetAnchor(anchorIndex), torqueFactor)

    def ApplyAngularForce(self, value: float):
        self.angularVelocity += value

    def Rotate(self, value: int):
        self.RotateRadians(math.radians(value))

    def RotateRadians(self, value: int):
        if value != 0:
            self.angle += value
            self.__transformUpdateRequired = True
            self.__aabbUpdateRequired = True
            self.__surfaceUpdateRequired = True

    def SetAngle(self, value: int):
        self.SetAngleRadians(math.radians(value))

    def SetAngleRadians(self, value: int):
        radiansValue = value
        if radiansValue != self.angle:
            self.angle = radiansValue
            self.__transformUpdateRequired = True
            self.__aabbUpdateRequired = True
            self.__surfaceUpdateRequired = True

    def GetAABB(self):
        if self.__aabbUpdateRequired:
            min = pg.Vector2(1e+20, 1e+20)
            max = pg.Vector2(-1e+20, -1e+20)

            if self.shape == "circle":
                min.x = self.position.x - self.radius
                min.y = self.position.y - self.radius

                max.x = self.position.x + self.radius
                max.y = self.position.y + self.radius

            else:
                vertices = self.GetTransformedVertices()

                for i in range(len(vertices)):
                    vert = vertices[i]

                    if vert.x < min.x: min.x = vert.x
                    if vert.x > max.x: max.x = vert.x
                    if vert.y < min.y: min.y = vert.y
                    if vert.y > max.y: max.y = vert.y

                if self.shape == "capsule":
                    min -= pg.Vector2(self.radius, self.radius)
                    max += pg.Vector2(self.radius, self.radius)

            self.__aabb = AABB(min, max)

            self.__aabbUpdateRequired = False
        
        return self.__aabb

    def GetTransformedVertices(self):
        if self.__transformUpdateRequired:
            transform = Transform(self.position, self.angle)

            for i in range(len(self.vertices)):
                vector = self.vertices[i]
                self.__transformedVertices[i] = TransformVector(vector, transform)

            self.__transformUpdateRequired = False
        
        return self.__transformedVertices
    
    def AddAnchor(self, anchor: pg.Vector2 | tuple):
        self.anchors.append(pg.Vector2(anchor))
        self.__transformedAnchors.append(pg.Vector2())
        self.__anchorsUpdateRequired = True
    
    def GetAnchor(self, index: int):
        if self.__anchorsUpdateRequired:
            transform = Transform(self.position, self.angle)

            for i in range(len(self.anchors)):
                self.__transformedAnchors[i] = TransformVector(self.anchors[i], transform)

            self.__anchorsUpdateRequired = False
        
        if index < 0 or index >= len(self.__transformedAnchors):
            return None

        return self.__transformedAnchors[index]

    def GetSurface(self):
        if not self.__surfaceUpdateRequired:
            return self.__transformedSurface
        
        self.__transformedSurface = pg.transform.rotate(self.surface, math.degrees(-self.angle))
        self.__surfaceUpdateRequired = False
        return self.__transformedSurface

    def Draw(self, screen: pg.Surface, cameraZoom: int, cameraPosition: pg.Vector2, showHitbox: bool = False, showSurfaces: bool = True):
        if self.visible:
            camPos = cameraPosition
            surface = self.GetSurface()
            position = self.position * cameraZoom - camPos

            if showSurfaces:
                screen.blit(surface, position - pg.Vector2(surface.get_size()) / 2)

            if self.shape == "circle":
                radius = self.radius * cameraZoom
                
                if showHitbox:
                    pg.draw.circle(screen, "white", position, radius, 2)
                    angleLine = pg.Vector2(radius, 0)
                    angleLine.rotate_rad_ip(self.angle)
                    pg.draw.line(screen, "red", position, position + angleLine, 2)

            if self.shape == "box":
                if showHitbox:
                    vertices = [i * cameraZoom - camPos for i in self.GetTransformedVertices()]
                    pg.draw.polygon(screen, "white", vertices, 2)

            if self.shape == "capsule":
                radius = self.radius * cameraZoom
                height = self.height * cameraZoom

                if showHitbox:
                    heightLine = pg.Vector2(0, height / 2)
                    heightLine.rotate_rad_ip(self.angle)
                    pg.draw.circle(screen, "white", position + heightLine, radius, 2)
                    pg.draw.circle(screen, "white", position - heightLine, radius, 2)

                    radiusLine = pg.Vector2(radius, 0)
                    radiusLine.rotate_rad_ip(self.angle)
                    vertices = [i * cameraZoom - camPos for i in self.GetTransformedVertices()]
                    pg.draw.line(screen, "white", vertices[0] + radiusLine, vertices[1] + radiusLine, 1)
                    pg.draw.line(screen, "white", vertices[0] - radiusLine, vertices[1] - radiusLine, 1)

                    pg.draw.line(screen, "red", position, position + radiusLine, 2)
                    # pg.draw.polygon(screen, "white", ((vertices - radiusLine), ()), 2)
                    # pg.draw.line(screen, "red", position, position + angleLine, 2)

    def SetSurface(self, surface: pg.Surface, transform: bool = True):
        if not transform:
            self.surface = surface.convert_alpha()
        
        else:
            if self.shape == "box":
                self.surface = pg.transform.scale(surface, (self.width, self.height)).convert_alpha()

            if self.shape == "circle":
                scaledSurface = pg.transform.scale(surface, (self.radius * 2, self.radius * 2))
                self.surface.fill((0, 0, 0, 0))
                pg.draw.circle(self.surface, "black", (self.radius, self.radius), self.radius)
                self.surface.blit(scaledSurface, (0, 0), special_flags=pg.BLEND_RGB_ADD)

            if self.shape == "capsule":
                scaledSurface = pg.transform.scale(surface, (self.radius * 2, self.radius * 2 + self.height))
                self.surface.fill((0, 0, 0, 0))
                pg.draw.rect(self.surface, "black", (0, 0, self.radius * 2, self.height + self.radius * 2), border_radius=self.radius)
                self.surface.blit(scaledSurface, (0, 0), special_flags=pg.BLEND_RGB_ADD)

    def CreateSurface(self, mainColor: tuple, outlineColor: tuple, outlineWidth: int):
        if self.shape == "box":
            self.surface.fill(mainColor)
            pg.draw.rect(self.surface, outlineColor, (0, 0, self.width, self.height), outlineWidth)

        if self.shape == "circle":
            pg.draw.circle(self.surface, mainColor, (self.radius, self.radius), self.radius)
            pg.draw.circle(self.surface, outlineColor, (self.radius, self.radius), self.radius, outlineWidth)

        if self.shape == "capsule":
            pg.draw.rect(self.surface, mainColor, (0, 0, self.radius * 2, self.height + self.radius * 2), border_radius=self.radius)
            pg.draw.rect(self.surface, outlineColor, (0, 0, self.radius * 2, self.height + self.radius * 2), outlineWidth, self.radius)

    def __str__(self):
        return f"Rigidbody(pos: {self.position}, angle: {self.angle}, size:[{self.width}, {self.height}])"
    
    def __repr__(self):
        return self.__str__()


def nearlyEqual(a: float, b: float, distance: float = 0.001):
    return abs(a - b) < distance

def vectorsNearlyEqual(a: pg.Vector2, b: pg.Vector2, distance: float = 0.005):
    return a.distance_squared_to(b) < distance**2


class Collisions:
    @staticmethod
    def __projectVertices(vertices: list[pg.Vector2], axis: pg.Vector2):
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
    def __projectCircle(center: pg.Vector2, radius: float, axis: pg.Vector2):
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
    def __projectCapsule(radius: float, axis: pg.Vector2, vertices: list[pg.Vector2]):
        direction = axis.normalize()
        directionAndRadius = direction * radius

        min = 1.7976931348623157e+308
        max = -1.7976931348623157e+308

        for i in range(len(vertices)):
            vert = vertices[i]
            projection1 = (vert - directionAndRadius).dot(axis)
            projection2 = (vert + directionAndRadius).dot(axis)

            projection = projection1 if projection1 < projection2 else projection2
            if projection < min:
                min = projection

            projection = projection1 if projection1 > projection2 else projection2
            if projection > max:
                max = projection

        if min > max:
            min, max = max, min

        return min, max

    @staticmethod
    def __closestPointOnPolygon(circleCenter: pg.Vector2, vertices: list[pg.Vector2]):
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
    def __closestPointOnPolygonToCapsule(verticesA: pg.Vector2, verticesB: list[pg.Vector2]):
        result = 0
        minDist = 1.7976931348623157e+308
        bestPoint = None

        for i in range(len(verticesB)):
            vector = verticesB[i]
            point, distance = Collisions.PointSegmentDistanceSquared(vector, verticesA[0], verticesA[1])
            if (distance < minDist):
                minDist = distance
                bestPoint = point
                result = i

        return result, bestPoint

    @staticmethod
    def PointSegmentDistanceSquared(point: pg.Vector2, a: pg.Vector2, b: pg.Vector2):
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
        if (first.shape == "circle" and second.shape == "capsule") or (second.shape == "circle" and first.shape == "capsule"):
            circle, capsule = (first, second) if first.shape == "circle" else (second, first)

            vertices = capsule.GetTransformedVertices()

            closestPoint, distance = Collisions.PointSegmentDistanceSquared(circle.position, vertices[0], vertices[1])

            direction = circle.position - closestPoint

            return closestPoint + direction.normalize() * capsule.radius, None, 1
        
        if (first.shape == "capsule" and second.shape == "box") or (second.shape == "capsule" and first.shape == "box"):
            capsule, box = (first, second) if first.shape == "capsule" else (second, first)
            verticesA = box.GetTransformedVertices()
            verticesB = capsule.GetTransformedVertices()

            contact1 = pg.Vector2()
            contact2 = pg.Vector2()
            contactCount = 0

            minDist = 1.0e+10
            for i in range(len(verticesA)):
                vert = verticesA[i]

                closestPoint, distance = Collisions.PointSegmentDistanceSquared(vert, verticesB[0], verticesB[1])

                if nearlyEqual(distance, minDist) and not vectorsNearlyEqual(closestPoint, contact1):
                    contact2 = closestPoint
                    contactCount = 2

                if distance < minDist:
                    minDist = distance
                    contact1 = vert
                    contactCount = 1

            for i in range(len(verticesB)):
                point = verticesB[i]

                for j in range(len(verticesA)):
                    closestPoint, distance = Collisions.PointSegmentDistanceSquared(point, verticesA[j], verticesA[(j + 1) % len(verticesA)])
                    
                    if nearlyEqual(distance, minDist) and not vectorsNearlyEqual(closestPoint, contact1):
                        contact2 = closestPoint
                        contactCount = 2

                    if distance < minDist:
                        minDist = distance
                        contact1 = closestPoint
                        contactCount = 1

            return contact1, contact2, contactCount

        if first.shape == "circle" and second.shape == "circle":
            return first.position + (second.position - first.position).normalize() * first.radius, None, 1
        
        if first.shape == "box" and second.shape == "box":
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

                    closestPoint, distance = Collisions.PointSegmentDistanceSquared(point, vertA, vertB)

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

                    closestPoint, distance = Collisions.PointSegmentDistanceSquared(point, vertA, vertB)

                    if nearlyEqual(distance, minDist) and not vectorsNearlyEqual(closestPoint, contact1):
                        contact2 = closestPoint
                        contactCount = 2

                    if distance < minDist:
                        minDist = distance
                        contactCount = 1
                        contact1 = closestPoint

            return contact1, contact2, contactCount
        
        if (first.shape == "circle" and second.shape == "box") or (second.shape == "circle" and first.shape == "box"):
            circle, box = (first, second) if first.shape == "circle" else (second, first)
            vertices = box.GetTransformedVertices()

            point = pg.Vector2()

            minDist = 1.0e+10
            for i in range(len(vertices)):
                vertA = vertices[i]
                vertB = vertices[(i + 1) % len(vertices)]

                contact, distance = Collisions.PointSegmentDistanceSquared(circle.position, vertA, vertB)

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

            minA, maxA = Collisions.__projectVertices(verticesA, axis)
            minB, maxB = Collisions.__projectVertices(verticesB, axis)

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

            minA, maxA = Collisions.__projectVertices(verticesA, axis)
            minB, maxB = Collisions.__projectVertices(verticesB, axis)

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

            minA, maxA = Collisions.__projectVertices(vertices, axis)
            minB, maxB = Collisions.__projectCircle(circlePosition, radius, axis)

            if minA >= maxB or minB >= maxA:
                return False, pg.Vector2(), 0
            
            axisDepth = min(maxB - minA, maxA - minB)

            if axisDepth < depth:
                depth = axisDepth
                normal = axis

        point = vertices[Collisions.__closestPointOnPolygon(circlePosition, vertices)]

        axis = point - circlePosition
        axis.normalize_ip()

        minA, maxA = Collisions.__projectVertices(vertices, axis)
        minB, maxB = Collisions.__projectCircle(circlePosition, radius, axis)

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
    def IntersectPolygonCapsule(verticesA: list[pg.Vector2], polygonPosition: pg.Vector2, verticesB: list[pg.Vector2], radius: float, height: float, circlePosition: pg.Vector2, isFirstABox: bool):
        isCollided, normal, depth = False, pg.Vector2(), 1.7976931348623157e+308

        for i in range(len(verticesA)):
            vertA = verticesA[i]
            vertB = verticesA[(i + 1) % len(verticesA)]

            edge = vertB - vertA
            axis = pg.Vector2(-edge.y, edge.x)
            axis.normalize_ip()

            minA, maxA = Collisions.__projectVertices(verticesA, axis)
            minB, maxB = Collisions.__projectCapsule(radius, axis, verticesB)

            if minA >= maxB or minB >= maxA:
                return False, pg.Vector2(), 0
            
            axisDepth = min(maxB - minA, maxA - minB)

            if axisDepth < depth:
                depth = axisDepth
                normal = axis

        idx, bestPoint = Collisions.__closestPointOnPolygonToCapsule(verticesB, verticesA) 
        point = verticesA[idx]

        axis = point - circlePosition
        axis.normalize_ip()

        minA, maxA = Collisions.__projectVertices(verticesA, axis)
        minB, maxB = Collisions.__projectCapsule(radius, axis, verticesB)

        if minA >= maxB or minB >= maxA:
            return False, pg.Vector2(), 0
        
        axisDepth = min(maxB - minA, maxA - minB)

        if axisDepth < depth:
            depth = axisDepth
            normal = axis

        isCollided = True

        direction = polygonPosition - bestPoint

        if (isFirstABox and direction.dot(normal) > 0) or (not isFirstABox and direction.dot(normal) < 0):
            normal = -normal
    
        return isCollided, normal, depth

    @staticmethod
    def IntersectCapsuleCircle(capsuleRadius: float, capsuleVertices: float, circlePosition: pg.Vector2, circleRadius: float):
        isCollided, normal, depth = False, pg.Vector2(), 0

        bestPoint, distance = Collisions.PointSegmentDistanceSquared(circlePosition, capsuleVertices[0], capsuleVertices[1])
        radius = capsuleRadius + circleRadius

        distance = math.sqrt(distance)
        
        if distance > radius:
            return isCollided, normal, depth

        normal = (circlePosition - bestPoint)

        normal.xy = (0, 1) if normal.xy == (0, 0) else normal.normalize()
        depth = radius - distance
        isCollided = True

        return isCollided, normal, depth
        
    @staticmethod
    def CollideBodies(first: Rigidbody, second: Rigidbody):
        isCollided, normal, depth = False, pg.Vector2(), 0

        if second in first.cannotCollideWith or first in second.cannotCollideWith:
            return isCollided, normal, depth
        
        elif (first.shape == "circle" and second.shape == "circle"):
            isCollided, normal, depth = Collisions.IntersectCircles(first.position, first.radius, second.position, second.radius)
        
        elif (first.shape == "box" and second.shape == "box"):
            isCollided, normal, depth = Collisions.IntersectPolygons(first.GetTransformedVertices(), first.position, second.GetTransformedVertices(), second.position)
            
        elif ((first.shape == "box" and second.shape == "circle") or (second.shape == "box" and first.shape == "circle")):
            polygon = second if second.shape == "box" else first
            circle = first if first.shape == "circle" else second

            isCollided, normal, depth = Collisions.IntersectPolygonCircle(polygon.GetTransformedVertices(), polygon.position, circle.radius, circle.position, first.shape == "box")

        elif ((first.shape == "capsule" and second.shape == "circle") or (second.shape == "capsule" and first.shape == "circle")):
            capsule = second if second.shape == "capsule" else first
            circle = first if first.shape == "circle" else second

            isCollided, normal, depth = Collisions.IntersectCapsuleCircle(capsule.radius, capsule.GetTransformedVertices(), circle.position, circle.radius) #, first.shape == "box"

        elif ((first.shape == "capsule" and second.shape == "box") or (second.shape == "capsule" and first.shape == "box")):
            polygon = second if second.shape == "box" else first
            capsule = first if first.shape == "capsule" else second

            isCollided, normal, depth = Collisions.IntersectPolygonCapsule(polygon.GetTransformedVertices(), polygon.position, capsule.GetTransformedVertices(), capsule.radius, capsule.height, capsule.position, first.shape == "box")

        return isCollided, normal, depth

    @staticmethod
    def CollideAABB(first: AABB, second: AABB):
        if (first.max.x <= second.min.x or second.max.x <= first.min.x or
            first.max.y <= second.min.y or second.max.y <= first.min.y):
            return False
        
        return True

    @staticmethod
    def CollidePoint(rigidbody: Rigidbody, point: pg.Vector2):
        if rigidbody.shape == "circle":
            return rigidbody.position.distance_squared_to(point) < rigidbody.radius**2
        
        elif rigidbody.shape == "capsule":
            vertices = rigidbody.GetTransformedVertices()
            return Collisions.PointSegmentDistanceSquared(point, vertices[0], vertices[1])[1] < rigidbody.radius**2
        
        elif rigidbody.shape == "box":
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
        self.bodyA = bodyA
        self.bodyB = bodyB
        self.normal = normal
        self.depth = depth
        self.contact1 = contacts[0]
        self.contact2 = contacts[1]
        self.contactCount = contacts[2]


class Resolver:
    @staticmethod
    def __ResolveCollisionsBasic(contact: CollisionManifold):
        first = contact.bodyA
        second = contact.bodyB
        normal = contact.normal

        relativeVelocity = second.linearVelocity - first.linearVelocity

        if relativeVelocity.dot(normal) > 0:
            return

        e = min(first.material.restitution, second.material.restitution)
        
        j = (-(1 + e) * relativeVelocity.dot(normal)) / (first.invMass + second.invMass)

        impulse = j * normal

        first.linearVelocity -= impulse * first.invMass
        second.linearVelocity += impulse * second.invMass

    @staticmethod
    def __ResolveCollisionsWithRotation(contact: CollisionManifold):
        first = contact.bodyA
        second = contact.bodyB
        normal = contact.normal

        contact1: pg.Vector2 = contact.contact1
        contact2: pg.Vector2 = contact.contact2
        contactCount = contact.contactCount

        e = min(first.material.restitution, second.material.restitution)

        contactList = [contact1, contact2]
        impulses: list[pg.Vector2] = []
        raList: list[pg.Vector2] = []
        rbList: list[pg.Vector2] = []

        for i in range(contactCount):
            ra = contactList[i] - first.position
            rb = contactList[i] - second.position

            raList.append(ra)
            rbList.append(rb)
            
            raPerp = pg.Vector2(-ra.y, ra.x)
            rbPerp = pg.Vector2(-rb.y, rb.x)

            angularLinearVelocityA = raPerp * first.angularVelocity
            angularLinearVelocityB = rbPerp * second.angularVelocity

            relativeVelocity = ((second.linearVelocity + angularLinearVelocityB) - 
                                (first.linearVelocity + angularLinearVelocityA))

            contactVelocityMag = relativeVelocity.dot(normal)

            if contactVelocityMag > 0:
                continue

            raPerpDotN = raPerp.dot(normal)
            rbPerpDotN = rbPerp.dot(normal)

            denom = (first.invMass + second.invMass + 
                        raPerpDotN**2 * first.invInertia + 
                        rbPerpDotN**2 * second.invInertia)

            j = -(1 + e) * contactVelocityMag
            j /= denom
            j /= contactCount

            impulse = j * normal
            impulses.append(impulse)

        for i in range(len(impulses)):
            impulse = impulses[i]

            first.linearVelocity += -impulse * first.invMass
            first.angularVelocity += -raList[i].cross(impulse) * first.invInertia

            second.linearVelocity += impulse * second.invMass
            second.angularVelocity += rbList[i].cross(impulse) * second.invInertia

    @staticmethod
    def __ResolveCollisionsWithRotationAndFriction(contact: CollisionManifold):
        first = contact.bodyA
        second = contact.bodyB
        normal = contact.normal

        contact1: pg.Vector2 = contact.contact1
        contact2: pg.Vector2 = contact.contact2
        contactCount = contact.contactCount

        e = min(first.material.restitution, second.material.restitution)

        staticFriction = (first.material.staticFriction + second.material.staticFriction) / 2
        dynamicFriction = (first.material.dynamicFriction + second.material.dynamicFriction) / 2

        contactList = [contact1, contact2]
        impulses: list[pg.Vector2] = []
        raList: list[pg.Vector2] = []
        rbList: list[pg.Vector2] = []
        jList: list[int] = []

        for i in range(contactCount):
            ra = contactList[i] - first.position
            rb = contactList[i] - second.position

            raList.append(ra)
            rbList.append(rb)
            
            raPerp = pg.Vector2(-ra.y, ra.x)
            rbPerp = pg.Vector2(-rb.y, rb.x)

            angularLinearVelocityA = raPerp * first.angularVelocity
            angularLinearVelocityB = rbPerp * second.angularVelocity

            relativeVelocity = ((second.linearVelocity + angularLinearVelocityB) - 
                                (first.linearVelocity + angularLinearVelocityA))

            contactVelocityMag = relativeVelocity.dot(normal)

            if contactVelocityMag > 0:
                continue

            raPerpDotN = raPerp.dot(normal)
            rbPerpDotN = rbPerp.dot(normal)

            denom = (first.invMass + second.invMass + 
                        raPerpDotN**2 * first.invInertia + 
                        rbPerpDotN**2 * second.invInertia)

            j = -(1 + e) * contactVelocityMag
            j /= denom
            j /= contactCount

            jList.append(j)

            impulse = j * normal
            impulses.append(impulse)

        for i in range(len(impulses)):
            impulse = impulses[i]

            first.linearVelocity += -impulse * first.invMass
            first.angularVelocity += -raList[i].cross(impulse) * first.invInertia

            second.linearVelocity += impulse * second.invMass
            second.angularVelocity += rbList[i].cross(impulse) * second.invInertia

        frictionImpulses = []

        for i in range(contactCount):
            ra = contactList[i] - first.position
            rb = contactList[i] - second.position

            raList[i] = ra
            rbList[i] = rb
            
            raPerp = pg.Vector2(-ra.y, ra.x)
            rbPerp = pg.Vector2(-rb.y, rb.x)

            angularLinearVelocityA = raPerp * first.angularVelocity
            angularLinearVelocityB = rbPerp * second.angularVelocity

            relativeVelocity = ((second.linearVelocity + angularLinearVelocityB) - 
                                (first.linearVelocity + angularLinearVelocityA))

            tangent = relativeVelocity - relativeVelocity.dot(normal) * normal

            if (vectorsNearlyEqual(tangent, pg.Vector2())):
                continue

            tangent.normalize_ip()

            raPerpDotT = raPerp.dot(tangent)
            rbPerpDotT = rbPerp.dot(tangent)

            denom = (first.invMass + second.invMass + 
                        raPerpDotT**2 * first.invInertia + 
                        rbPerpDotT**2 * second.invInertia)

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

            first.linearVelocity += -impulse * first.invMass
            first.angularVelocity += -raList[i].cross(impulse) * first.invInertia

            second.linearVelocity += impulse * second.invMass
            second.angularVelocity += rbList[i].cross(impulse) * second.invInertia

    resolvingCollisionsFunctions = [__ResolveCollisionsBasic, __ResolveCollisionsWithRotation, __ResolveCollisionsWithRotationAndFriction]

    @staticmethod
    def SeparateBodies(first: Rigidbody, second: Rigidbody, mtv: pg.Vector2):
        if first.isStatic:
            second.Move(mtv)
        elif second.isStatic:
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

                    if (mainBody.isStatic and otherBody.isStatic) or not Collisions.CollideAABB(mainAABB, otherAABB):
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
    def UpdateBodies(screenHeight: int, dt: float, gravity: pg.Vector2, bodies: list[Rigidbody]):
        for body in bodies:
            body.Update(dt, gravity)

            if not body.isStatic and body.GetAABB().min.y >= screenHeight:
                bodies.remove(body)
                continue


class Joint:
    def __init__(self, bodyA: Rigidbody, bodyB: Rigidbody, anchorAIndex: int, anchorBIndex: int):
        self.bodyA = bodyA
        self.bodyB = bodyB
        self.anchorAIndex = anchorAIndex
        self.anchorBIndex = anchorBIndex

    def Update(self, dt: float): pass


class ForceJoint(Joint):
    def __init__(self, bodyA: Rigidbody, bodyB: Rigidbody, anchorAIndex: int, anchorBIndex: int, strength: float):
        super().__init__(bodyA, bodyB, anchorAIndex, anchorBIndex)

        self.strength = strength

    def Update(self, dt: float):
        a = self.bodyA.GetAnchor(self.anchorAIndex)
        b = self.bodyB.GetAnchor(self.anchorBIndex)
        direction = (b - a).normalize() * self.strength * 0.1
        
        if not self.bodyA.isStatic:
            self.bodyA.ApplyForceAtPoint(direction, b, 1)
        if not self.bodyB.isStatic:
            self.bodyB.ApplyForceAtPoint(-direction, a, 1)


class SpringJoint(Joint):
    def __init__(self, bodyA: Rigidbody, bodyB: Rigidbody, anchorAIndex: int, anchorBIndex: int, restLength: float, springConstant: float):
        super().__init__(bodyA, bodyB, anchorAIndex, anchorBIndex)
        self.springConstant = springConstant
        self.restLength = restLength

    def Update(self, dt: float):
        anchorAPos = self.bodyA.GetAnchor(self.anchorAIndex)
        anchorBPos = self.bodyB.GetAnchor(self.anchorBIndex)

        direction = anchorBPos - anchorAPos
        distance = direction.length()
        restDistance = distance - self.restLength
        forceMagnitude = restDistance * self.restLength * self.springConstant * 0.001

        force = direction.normalize() * forceMagnitude
        if not self.bodyA.isStatic:
            self.bodyA.ApplyForceAtPoint(force / 2, anchorAPos)

        if not self.bodyB.isStatic:                                
            self.bodyB.ApplyForceAtPoint(-force / 2, anchorBPos)


# def ResolveCollisionHingeJoint(contact: CollisionManifold, penetrationPoint: pg.Vector2):
#     penetrationToCentroidA = penetrationPoint - contact.bodyA.position
#     penetrationToCentroidB = penetrationPoint - contact.bodyB.position
        
#     angularVelocityPenetrationCentroidA = pg.Vector2(-contact.bodyA.angularVelocity * penetrationToCentroidA.y, contact.bodyA.angularVelocity * penetrationToCentroidA.x)
#     angularVelocityPenetrationCentroidB = pg.Vector2(-contact.bodyB.angularVelocity * penetrationToCentroidB.y, contact.bodyB.angularVelocity * penetrationToCentroidB.x)

#     relativeVelocityA = contact.bodyA.linearVelocity + angularVelocityPenetrationCentroidA
#     relativeVelocityB = contact.bodyB.linearVelocity + angularVelocityPenetrationCentroidB
                        
#     relativeVel = relativeVelocityB - relativeVelocityA
#     velocityInNormal = relativeVel.dot(contact.normal)	

#     if velocityInNormal > 0: return
    
#     e = min(contact.bodyA.material.restitution, contact.bodyB.material.restitution)	
#     pToCentroidCrossNormalA = penetrationToCentroidA.cross(contact.normal)
#     pToCentroidCrossNormalB = penetrationToCentroidB.cross(contact.normal)
    
#     invMassSum = contact.bodyA.invMass + contact.bodyB.invMass

#     bodyAInvInertia = contact.bodyA.invInertia
#     bodyBInvInertia = contact.bodyB.invInertia
#     crossNSum  = pToCentroidCrossNormalA * pToCentroidCrossNormalA * bodyAInvInertia + pToCentroidCrossNormalB * pToCentroidCrossNormalB * bodyBInvInertia

#     j = -(1 + e ) * velocityInNormal
#     j /= (invMassSum + crossNSum)

#     impulseVector = contact.normal * j
    
#     contact.bodyA.linearVelocity -= impulseVector * contact.bodyA.invMass
#     contact.bodyB.linearVelocity -= impulseVector * contact.bodyB.invMass	
#     contact.bodyA.angularVelocity += -pToCentroidCrossNormalA * j * bodyAInvInertia		
#     contact.bodyB.angularVelocity += pToCentroidCrossNormalB * j * bodyBInvInertia	

#     # Frictional impulse
#     velocityInNormalDirection = contact.normal * relativeVel.dot(contact.normal)
#     tangent = velocityInNormalDirection - relativeVel
#     tangent *= -1
#     minFriction = min(contact.bodyA.material.dynamicFriction, contact.bodyB.material.dynamicFriction)
#     if(tangent.x > 0.00001 or tangent.y > 0.00001):
#         tangent.normalize_ip()
    
#     pToCentroidCrossTangentA = penetrationToCentroidA.cross(tangent)
#     pToCentroidCrossTangentB = penetrationToCentroidB.cross(tangent)

#     crossSumTangent = pToCentroidCrossTangentA * pToCentroidCrossTangentA * bodyAInvInertia + pToCentroidCrossTangentB * pToCentroidCrossTangentB * bodyBInvInertia
#     frictionalImpulse = -(1 + e ) * relativeVel.dot(tangent) * minFriction
#     frictionalImpulse /= (invMassSum + crossSumTangent)
#     if frictionalImpulse > j:
#         frictionalImpulse = j

#     frictionalImpulseVector = tangent * frictionalImpulse
    
#     contact.bodyA.linearVelocity -= frictionalImpulseVector * contact.bodyA.invMass
#     contact.bodyB.linearVelocity -= frictionalImpulseVector * contact.bodyB.invMass	
    
#     contact.bodyA.angularVelocity += -pToCentroidCrossTangentA * frictionalImpulse * bodyAInvInertia		
#     contact.bodyB.angularVelocity += pToCentroidCrossTangentB * frictionalImpulse * bodyBInvInertia
	

class HingeJoint(Joint):
    def __init__(self, bodyA: Rigidbody, bodyB: Rigidbody, anchorAIndex: int, anchorBIndex: int, length: float = -1, strength: float = 5):
        super().__init__(bodyA, bodyB, anchorAIndex, anchorBIndex)

        self.bodyA.cannotCollideWith.append(self.bodyB)

        anchorA = self.bodyA.GetAnchor(self.anchorAIndex)
        anchorB = self.bodyB.GetAnchor(self.anchorBIndex)

        self.materialA = self.bodyA.material
        self.materialB = self.bodyB.material

        self.initialLength = (anchorA - anchorB).length() if length < 0 else length

        self.relativeAngle = self.bodyB.angle - self.bodyA.angle

        self.strength = pg.math.clamp(strength, 0, 100000)

    def Update(self, dt: float):
        # self.bodyA.material = Material(0, 0.0001, 0.0001, 0.0001)
        # self.bodyB.material = Material(0, 0.0001, 0.0001, 0.0001)

        anchorA = self.bodyA.GetAnchor(self.anchorAIndex)
        anchorB = self.bodyB.GetAnchor(self.anchorBIndex)

        direction = anchorB - anchorA
        distance = direction.length()
        if distance < 0.001:
            # self.bodyA.material = self.materialA
            # self.bodyB.material = self.materialB
            return

        normal = direction.normalize()

        # if (self.initialLength - distance) < -3:
        contact = CollisionManifold(self.bodyB, self.bodyA, normal, self.initialLength - distance, (None, None, 0))
        Resolver.SeparateBodies(self.bodyA, self.bodyB, contact.normal * contact.depth * 0.5)
        # Resolver.resolvingCollisionsFunctions[0](contact)
            # return

        if not self.bodyA.isStatic:
            self.bodyA.ApplyForceAtAnchor(-normal * (self.initialLength - distance) * self.strength, self.anchorAIndex)
            
        if not self.bodyB.isStatic:
            self.bodyB.ApplyForceAtAnchor(normal * (self.initialLength - distance) * self.strength, self.anchorBIndex)

        # Resolver.SeparateBodies(self.bodyA, self.bodyB, contact.normal * contact.depth * 0.5)
        # Resolver.resolvingCollisionsFunctions[0](contact)

        # self.bodyA.material = self.materialA
        # self.bodyB.material = self.materialB


class FixedJointWIP(HingeJoint):
    def __init__(self, bodyA: Rigidbody, bodyB: Rigidbody, anchorAIndex: int, anchorBIndex: int, strength: float = 5, angularStrength: float = 5):
        super().__init__(bodyA, bodyB, anchorAIndex, anchorBIndex, 0, strength)

        self.angularStrength = angularStrength

    def Update(self, dt: float):
        super().Update(dt)
        
        # if abs(self.relativeAngle - (self.bodyA.angle - self.bodyB.angle)) > 1:
        # if not self.bodyB.isStatic:
        #     self.bodyB.RotateRadians(-(self.relativeAngle - (self.bodyA.angle - self.bodyB.angle)))
        # if not self.bodyA.isStatic:
        #     self.bodyA.RotateRadians((self.relativeAngle - (self.bodyA.angle - self.bodyB.angle)))

            # return

        if not self.bodyB.isStatic:
            self.bodyB.angularVelocity += (self.relativeAngle - (self.bodyB.angle - self.bodyA.angle)) * self.angularStrength
        if not self.bodyA.isStatic:
            self.bodyA.angularVelocity += (self.relativeAngle - (self.bodyA.angle - self.bodyB.angle)) * self.angularStrength


class World:
    def __init__(self):
        self.minBodySize = 0.01**2 * 30
        self.maxBodySize = 64**2 * 30

        self.minDensity = 0.5
        self.maxDensity = 23

        self.minIterations = 1
        self.MaxIterations = 128

        self.__gravity = pg.Vector2(0, 200)#  * 37.8

        self.__bodies: list[Rigidbody] = []

        self.__joints: list[Joint] = []

        self.__contactPairs: list[tuple[int, int]] = []

        self.bodyCount = 0

        self.cameraZoom = 1
        self.cameraPosition = pg.Vector2()

        self.showHitbox = True
        self.showSurfaces = True

    def CameraLookAtPoint(self, point: pg.Vector2, smooth: float):
        smooth = pg.math.clamp(smooth, 1, 100)
        self.cameraPosition += (point - self.cameraPosition) / smooth
    
    def ZoomCamera(self, value: int):
        self.cameraZoom += value

    def MoveCamera(self, value: pg.Vector2):
        self.cameraPosition += value

    def RayCast(self, origin: pg.Vector2, direction: float, maxLength: float, step: int, dontCollideWith: list[Rigidbody] = []):
        length = 0
        point = origin.copy()

        for _ in range(maxLength // step):
            point.x += step * math.cos(math.radians(direction + 90))
            point.y -= step * math.sin(math.radians(direction + 90))
            length += step

            for body in self.__bodies:
                collide = False
                while body not in dontCollideWith and Collisions.CollidePoint(body, point):
                    collide = True
                    point.x -= 1 * math.cos(math.radians(direction + 90))
                    point.y += 1 * math.sin(math.radians(direction + 90))
                    length -= 1

                if collide:
                    return point, body

        return point, None
    
    def AddJoint(self, joint: Joint):
        self.__joints.append(joint)

    def RemoveJoint(self, joint: Joint):
        if joint not in self.__joints:
            return False
        
        self.__joints.remove(joint)
        return True

    def RemoveJointByIndex(self, index: int):
        if index < 0 or index >= len(self.__joints):
            return False
        
        self.__joints.pop(index)
        return True
    
    def CreateCapsuleBody(self, material: Material, isStatic: bool, radius: float, height: float, freezeRotation=False, createClass=Rigidbody):
        area = radius ** 2 * math.pi + height * radius * 2
        if area < self.minBodySize:
            error(f"Радиус окружности слишком маленький. Минимальная площадь - {self.minBodySize}.")
        
        if area > self.maxBodySize:
            error(f"Радиус окружности слишком большой. Максимальная площадь - {self.maxBodySize}.")
        
        if material.density < self.minDensity:
            error(f"Плотность материала слишком маленькая. Минимальная плотность - {self.minDensity}.")
        
        if material.density > self.maxDensity:
            error(f"Плотность материала слишком большая. Максимальная плотность - {self.maxDensity}.")

        if not issubclass(createClass, Rigidbody):
            error(f"Класс для создания должен являться Rigidbody или наследоваться от него.")

        mass = (area / 37.93**2 / 10000) * material.density

        body: Rigidbody = createClass(mass, area, material, isStatic, radius, 0, height, "capsule")
        body.inertia = 0.5 * mass * radius**2 + (1 / 12) * mass * ((radius * 2)**2 + height**2)

        if not body.isStatic:
            if not freezeRotation:
                body.invInertia = 1 / body.inertia
            body.invMass = 1 / body.mass
            
        self.__bodies.append(body)

        return body

    def CreateCircleBody(self, material: Material, isStatic: bool, radius: float, freezeRotation=False, createClass=Rigidbody):
        area = radius ** 2 * math.pi
        if area < self.minBodySize:
            error(f"Радиус окружности слишком маленький. Минимальная площадь - {self.minBodySize}.")
        
        if area > self.maxBodySize:
            error(f"Радиус окружности слишком большой. Максимальная площадь - {self.maxBodySize}.")
        
        if material.density < self.minDensity:
            error(f"Плотность материала слишком маленькая. Минимальная плотность - {self.minDensity}.")
        
        if material.density > self.maxDensity:
            error(f"Плотность материала слишком большая. Максимальная плотность - {self.maxDensity}.")

        if not issubclass(createClass, Rigidbody):
            error(f"Класс для создания должен являться Rigidbody или наследоваться от него.")

        mass = (area / 37.93**2 / 10000) * material.density

        body: Rigidbody = createClass(mass, area, material, isStatic, radius, 0, 0, "circle")
        body.inertia = 0.5 * mass * radius**2

        if not body.isStatic:
            if not freezeRotation:
                body.invInertia = 1 / body.inertia
            body.invMass = 1 / body.mass
            
        self.__bodies.append(body)

        return body
    
    def CreateBoxBody(self, material: Material, isStatic: bool, width: float, height: float, freezeRotation=False, createClass=Rigidbody):
        area = width * height
        if area < self.minBodySize:
            error(f"Ширина и высота слишком маленькие. Минимальная площадь - {self.minBodySize}.")
        
        if area > self.maxBodySize:
            error(f"Ширина и высота слишком большие. Максимальная площадь - {self.maxBodySize}.")
        
        if material.density < self.minDensity:
            error(f"Плотность тела слишком маленькая. Минимальная плотность - {self.minDensity}.")
        
        if material.density > self.maxDensity:
            error(f"Плотность тела слишком маленькая. Максимальная плотность - {self.maxDensity}.")

        if not issubclass(createClass, Rigidbody):
            error(f"Класс для создания должен являться Rigidbody или наследоваться от него.")
        
        mass = (area / 37.93**2 / 10000) * material.density

        body: Rigidbody = createClass(mass, area, material, isStatic, 0, width, height, "box")
        body.inertia = (1 / 12) * mass * (width**2 + height**2)

        if not body.isStatic:
            if not freezeRotation:
                body.invInertia = 1 / body.inertia
            body.invMass = 1 / body.mass
        
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
        self.bodyCount = len(self.__bodies)
        iterations = pg.math.clamp(iterations, self.minIterations, self.MaxIterations)
        dt /= iterations
        for _ in range(iterations):
            self.__contactPairs.clear()

            Resolver.UpdateBodies(screen.get_height(), dt, self.__gravity, self.__bodies)
            Resolver.BroadPhase(self.__bodies, self.__contactPairs)
            Resolver.NarrowPhase(self.__bodies, self.__contactPairs, 2)

            for joint in self.__joints:
                joint.Update(dt)

    def Render(self, screen: pg.Surface):
        for i in range(len(self.__bodies)):
            self.__bodies[i].Draw(screen, self.cameraZoom, self.cameraPosition, self.showHitbox, self.showSurfaces)