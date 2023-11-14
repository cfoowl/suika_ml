from physic_engine.body import Body
from common.shape import Circle, Polygon
from common.math import Vec2
from math import sqrt


def CircletoCircle(m, a: Body, b: Body):
    A = a.shape
    B = b.shape

    # Calculate translational vector, which is normal
    normal = b.position - a.position

    dist_sqr = normal.LenSqr()
    radius = A.radius + B.radius

    # Not in contact
    if dist_sqr >= radius * radius:
        m.contact_count = 0
        return

    distance = sqrt(dist_sqr)

    m.contact_count = 1

    if distance == 0.0:
        m.penetration = A.radius
        m.normal = Vec2(1, 0)
        m.contacts[0] = a.position
    else:
        m.penetration = radius - distance
        m.normal = normal / distance
        m.contacts[0] = m.normal * A.radius + a.position


def CircletoPolygon(m, a: Body, b: Body):
    A = a.shape  # Circle
    B = b.shape  # Polygon

    m.contact_count = 0

    # Transform circle center to Polygon model space
    center = a.position
    center = B.u.Transpose() * (center - b.position)

    # Find edge with minimum penetration
    # Exact concept as using support points in Polygon vs Polygon
    separation = -float("inf")
    faceNormal = 0
    for i in range(B.m_vertexCount):
        s = Dot(B.m_normals[i], center - B.m_vertices[i])

        if s > A.radius:
            return
        if s > separation:
            separation = s
            faceNormal = i

    # Grab face's vertices
    v1 = B.m_vertices[faceNormal]
    i2 = faceNormal + 1 if faceNormal + 1 < B.m_vertexCount else 0
    v2 = B.m_vertices[i2]

    # Check to see if center is within polygon
    if separation < EPSILON:
        m.contact_count = 1
        m.normal = -(B.u * B.m_normals[faceNormal])
        m.contacts[0] = m.normal * A.radius + a.position
        m.penetration = A.radius
        return

    # Determine which voronoi region of the edge center of circle lies within
    dot1 = Dot(center - v1, v2 - v1)
    dot2 = Dot(center - v2, v1 - v2)
    m.penetration = A.radius - separation

    # Closest to v1
    if dot1 <= 0.0:
        if DistSqr(center, v1) > A.radius * A.radius:
            return
        m.contact_count = 1
        n = v1 - center
        n = B.u * n
        n.Normalize()
        m.normal = n
        v1 = B.u * v1 + b.position
        m.contacts[0] = v1

    # Closest to v2
    elif dot2 <= 0.0:
        if DistSqr(center, v2) > A.radius * A.radius:
            return

        m.contact_count = 1
        n = v2 - center
        v2 = B.u * v2 + b.position
        m.contacts[0] = v2
        n = B.u * n
        n.Normalize()
        m.normal = n

    # Closest to face
    else:
        n = B.m_normals[faceNormal]
        if Dot(center - v1, n) > A.radius:
            return

        n = B.u * n
        m.normal = -n
        m.contacts[0] = m.normal * A.radius + a.position
        m.contact_count = 1


def PolygontoCircle(m, a: Body, b: Body):
    CircletoPolygon(m, b, a)
    m.normal = -m.normal


def FindAxisLeastPenetration(faceIndex, A: Polygon, B: Polygon):
    bestDistance = -float("inf")
    bestIndex = 0

    for i in range(A.m_vertexCount):
        # Retrieve a face normal from A
        n = A.m_normals[i]
        nw = A.u * n

        # Transform face normal into B's model space
        buT = B.u.Transpose()
        n = buT * nw

        # Retrieve support point from B along -n
        s = B.GetSupport(-n)

        # Retrieve vertex on face from A, transform into
        # B's model space
        v = A.m_vertices[i]
        v = A.u * v + A.body.position
        v -= B.body.position
        v = buT * v

        # Compute penetration distance (in B's model space)
        d = Dot(n, s - v)

        # Store greatest distance
        if d > bestDistance:
            bestDistance = d
            bestIndex = i

    faceIndex[0] = bestIndex
    return bestDistance


def FindIncidentFace(v: Vec2, RefPoly: Polygon, IncPoly: Polygon, referenceIndex: int):
    referenceNormal = RefPoly.m_normals[referenceIndex]

    # Calculate normal in incident's frame of reference
    referenceNormal = RefPoly.u * referenceNormal  # To world space
    referenceNormal = (
        IncPoly.u.Transpose() * referenceNormal
    )  # To incident's model space

    # Find most anti-normal face on incident polygon
    incidentFace = 0
    minDot = float("inf")
    for i in range(IncPoly.m_vertexCount):
        dot = Dot(referenceNormal, IncPoly.m_normals[i])
        if dot < minDot:
            minDot = dot
            incidentFace = i

    v[0] = IncPoly.u * IncPoly.m_vertices[incidentFace] + IncPoly.body.position
    incidentFace = incidentFace + 1 if incidentFace + 1 < IncPoly.m_vertexCount else 0
    v[1] = IncPoly.u * IncPoly.m_vertices[incidentFace] + IncPoly.body.position


def Clip(n: Vec2, c: float, face):
    sp = 0
    out = [Vec2(face[0]), Vec2(face[1])]

    # Retrieve distances from each endpoint to the line
    # d = ax + by - c
    d1 = Dot(n, face[0]) - c
    d2 = Dot(n, face[1]) - c

    # If negative (behind plane) clip
    if d1 <= 0.0:
        out[sp] = face[0]
        sp += 1
    if d2 <= 0.0:
        out[sp] = face[1]
        sp += 1

    # If the points are on different sides of the plane
    if d1 * d2 < 0.0:  # less than to ignore -0.0
        # Push intersection point
        alpha = d1 / (d1 - d2)
        out[sp] = face[0] + alpha * (face[1] - face[0])
        sp += 1

    # Assign our new converted values
    face[0] = out[0]
    face[1] = out[1]

    assert sp != 3

    return sp


def PolygontoPolygon(m, a: Body, b: Body):
    A = a.shape  # Polygon A
    B = b.shape  # Polygon B

    m.contact_count = 0

    # Check for a separating axis with A's face planes
    faceA = [0]
    penetrationA = FindAxisLeastPenetration(faceA, A, B)
    if penetrationA >= 0.0:
        return

    # Check for a separating axis with B's face planes
    faceB = [0]
    penetrationB = FindAxisLeastPenetration(faceB, B, A)
    if penetrationB >= 0.0:
        return

    referenceIndex = 0
    flip = False  # Always point from a to b

    RefPoly = None  # Reference
    IncPoly = None  # Incident

    # Determine which shape contains reference face
    if BiasGreaterThan(penetrationA, penetrationB):
        RefPoly = A
        IncPoly = B
        referenceIndex = faceA[0]
        flip = False
    else:
        RefPoly = B
        IncPoly = A
        referenceIndex = faceB[0]
        flip = True

    # World space incident face
    incidentFace = [Vec2(0), Vec2(0)]
    FindIncidentFace(incidentFace, RefPoly, IncPoly, referenceIndex)

    # Setup reference face vertices
    v1 = RefPoly.m_vertices[referenceIndex]
    referenceIndex = (
        referenceIndex + 1 if referenceIndex + 1 != RefPoly.m_vertexCount else 0
    )
    v2 = RefPoly.m_vertices[referenceIndex]

    # Transform vertices to world space
    v1 = RefPoly.u * v1 + RefPoly.body.position
    v2 = RefPoly.u * v2 + RefPoly.body.position

    # Calculate reference face side normal in world space
    sidePlaneNormal = v2 - v1
    sidePlaneNormal.Normalize()

    # Orthogonalize
    refFaceNormal = Vec2(sidePlaneNormal.y, -sidePlaneNormal.x)

    refC = Dot(refFaceNormal, v1)
    negSide = -Dot(sidePlaneNormal, v1)
    posSide = Dot(sidePlaneNormal, v2)

    # Clip incident face to reference face side planes
    if Clip(-sidePlaneNormal, negSide, incidentFace) < 2:
        return  # Due to floating point error, possible to not have required points

    if Clip(sidePlaneNormal, posSide, incidentFace) < 2:
        return

    # Flip
    m.normal = -refFaceNormal if flip else refFaceNormal

    # Keep points behind reference face
    cp = 0  # Contact point counter
    separation = Dot(refFaceNormal, incidentFace[0]) - refC
    if separation <= 0.0:
        m.contacts[cp] = incidentFace[0]
        m.penetration = -separation
        cp += 1
    else:
        m.penetration = 0

    separation = Dot(refFaceNormal, incidentFace[1]) - refC
    if separation <= 0.0:
        m.contacts[cp] = incidentFace[1]

        m.penetration += -separation
        cp += 1

        # Average penetration
        m.penetration /= cp

    m.contact_count = cp


collision_dispatch_table = {
    ("Circle", "Circle"): CircletoCircle,
    ("Circle", "Polygon"): CircletoPolygon,
    ("Polygon", "Circle"): PolygontoCircle,
    ("Polygon", "Polygon"): PolygontoPolygon,
}
