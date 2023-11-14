from common.math import *
import math
import pygame as pg


class Shape:
    def __init__(self):
        self.body = None

    def GetType(self):
        pass

    def ComputeMass(self, density):
        pass

    def SetOrientation(self, radians):
        pass

    def Draw(self):
        pass


class Circle(Shape):
    def __init__(self, radius):
        super().__init__()
        self.radius = radius

    def GetType(self):
        return "Circle"

    def ComputeMass(self, density):
        self.body.mass = PI * self.radius * self.radius * density
        self.body.invMass = 1.0 / self.body.mass if self.body.mass else 0.0
        self.body.inertia = self.body.mass * self.radius * self.radius
        self.body.invInertia = 1.0 / self.body.inertia if self.body.inertia else 0.0

    def SetOrientation(self, radians):
        pass

    def Draw(self, screen):
        # Draw the outline of the circle
        position = (self.body.position.x, self.body.position.y)
        pg.draw.circle(
            screen, (255, 255, 255), position, self.radius, 1
        )  # 1 pixel outline

        # Draw the orientation line
        end_x = self.body.position.x + self.radius * math.cos(self.body.orientation)
        end_y = self.body.position.y + self.radius * math.sin(self.body.orientation)
        pg.draw.line(screen, (255, 0, 0), position, (end_x, end_y), 1)


class Polygon(Shape):
    maxPolyVertexCount = 64

    def __init__(self):
        super().__init__()
        self.u = Mat2()  # orientation matrix
        self.m_vertexCount = 0
        self.m_vertices = [Vec2(0, 0) for _ in range(self.maxPolyVertexCount)]
        self.m_normals = [Vec2(0, 0) for _ in range(self.maxPolyVertexCount)]

    def ComputeMass(self, density):
        c = Vec2(0.0, 0.0)  # centroid
        area = 0.0
        I = 0.0
        k_inv3 = 1.0 / 3.0

        for i1 in range(self.m_vertexCount):
            p1 = self.m_vertices[i1]
            i2 = i1 + 1 if i1 + 1 < self.m_vertexCount else 0
            p2 = self.m_vertices[i2]

            D = Cross(p1, p2)
            triangleArea = 0.5 * D

            area += triangleArea

            c += triangleArea * k_inv3 * (p1 + p2)

            intx2 = p1.x * p1.x + p2.x * p1.x + p2.x * p2.x
            inty2 = p1.y * p1.y + p2.y * p1.y + p2.y * p2.y
            I += (0.25 * k_inv3 * D) * (intx2 + inty2)

        x *= 1 / area

        for i in range(self.m_vertexCount):
            self.m_vertices[i] -= c

        self.body.mass = density * area
        self.body.invMass = 1.0 / self.body.mass if self.body.mass else 0.0
        self.body.inertia = I * density
        self.body.invInertia = 1.0 / self.body.inertia if self.body.inertia else 0.0

    def SetOrientation(self, radians):
        self.u.Set(radians)

    def GetType(self):
        return "Polygon"

    def SetBox(self, hw, hh):
        """
        hh : half height
        hw : half width
        """
        self.m_vertexCount = 4
        self.m_vertices[0].Set(-hw, -hh)
        self.m_vertices[1].Set(hw, -hh)
        self.m_vertices[2].Set(hw, hh)
        self.m_vertices[3].Set(-hw, hh)
        self.m_normals[0].Set(0.0, -1.0)
        self.m_normals[1].Set(1.0, 0.0)
        self.m_normals[2].Set(0.0, 1.0)
        self.m_normals[3].Set(-1.0, 0.0)

    def Set(self, vertices, count):
        # No hulls with less than 3 vertices (ensure actual polygon)
        assert count > 2 and count <= self.maxPolyVertexCount
        count = min(count, self.maxPolyVertexCount)

        # Find the right most point on the hull
        rightMost = 0
        highestXCoord = vertices[0].x
        for i in range(count):
            x = vertices[i].x
            if x > highestXCoord:
                highestXCoord = x
                rightMost = i
            # If matching x then take farthest negative y
            elif x == highestXCoord:
                if vertices[i].y < vertices[rightMost].y:
                    rightMost = i

        hull = [0 for _ in range(self.maxPolyVertexCount)]
        outCount = 0
        indexHull = rightMost

        while True:
            hull[outCount] = indexHull
            nextHullIndex = 0
            for i in range(1, count):
                if nextHullIndex == indexHull:
                    nextHullIndex = i
                    continue

            e1 = vertices[nextHullIndex] - vertices[hull[outCount]]
            e2 = vertices[i] - vertices[hull[outCount]]
            c = Cross(e1, e2)
            if c < 0.0:
                nextHullIndex = i

            if c == 0.0 and e2.LenSqr() > e1.LenSqr():
                nextHullIndex = i

            outCount += 1
            indexHull = nextHullIndex

            if nextHullIndex == rightMost:
                self.m_vertexCount = outCount
                break

            for i in range(0, self.m_vertexCount):
                self.m_vertices[i] = vertices[hull[i]]

            for i1 in range(0, self.m_vertexCount):
                i2 = i1 + 1 if i1 + 1 < self.m_vertexCount else 0
                face = self.m_vertices[i2] - self.m_vertices[i1]

                assert face.LenSqr() > EPSILON * EPSILON

                self.m_normals[i1] = Vec2(face.y, -face.x)
                self.m_normals[i1].normalize

    def GetSupport(self, dir):
        bestProjection = -float("inf")
        bestVertex = None
        for i in range(self.m_vertexCount):
            v = self.m_vertices[i]
            projection = Dot(v, dir)

            if projection > bestProjection:
                bestVertex = v
                bestProjection = projection

        return bestVertex
