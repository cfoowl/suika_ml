from common.math import *
from physic_engine.body import Body
from common.shape import Shape
from physic_engine.collision import collision_dispatch_table
from math import sqrt


class Manifold:
    def __init__(self, a: Body, b: Body):
        self.A = a
        self.B = b

        self.penetration = 0  # Depth of penetration from collision
        self.normal = Vec2(0, 0)  # From A to B
        self.contacts = [None, None]  # Points of contact during collision
        self.contact_count = 0  # Number of contacts that occured during collision
        self.e = 0  # Mixed restitution
        self.df = 0  # Mixed dynamic friction
        self.sf = 0  # Mixed static friction

    def Solve(self):
        collision_dispatch_table.get((self.A.shape.GetType(), self.B.shape.GetType()))(
            self, self.A, self.B
        )

    def Initialize(self, dt, gravity):
        # Calculate average restitution
        self.e = min(self.A.restitution, self.B.restitution)

        # Calculate static and dynamic friction
        self.sf = sqrt(self.A.staticFriction * self.B.staticFriction)
        self.df = sqrt(self.A.dynamicFriction * self.B.dynamicFriction)

        for i in range(self.contact_count):
            ra = self.contacts[i] - self.A.position
            rb = self.contacts[i] - self.B.position

            rv = (
                self.B.velocity
                + Cross(self.B.angularVelocity, rb)
                - self.A.velocity
                - Cross(self.A.angularVelocity, ra)
            )

            if rv.LenSqr() < (gravity * dt).LenSqr() + EPSILON:
                self.e = 0.0

    def ApplyImpulse(self):
        # Early out and positional correct if both objects have infinite mass
        if Equal(self.A.invMass + self.B.invMass, 0):
            self.InfiniteMassCorrection()
            return

        for i in range(self.contact_count):
            # Calculate radii from COM to contact
            ra = self.contacts[i] - self.A.position
            rb = self.contacts[i] - self.B.position

            # Relative velocity
            rv = (
                self.B.velocity
                + Cross(self.B.angularVelocity, rb)
                - self.A.velocity
                - Cross(self.A.angularVelocity, ra)
            )

            # Relative velocity along the normal
            contactVel = Dot(rv, self.normal)

            # Do not resolve if velocities are separating
            if contactVel > 0:
                return

            raCrossN = Cross(ra, self.normal)
            rbCrossN = Cross(rb, self.normal)
            invMassSum = (
                self.A.invMass
                + self.B.invMass
                + Sqr(raCrossN) * self.A.invInertia
                + Sqr(rbCrossN) * self.B.invInertia
            )

            # Calculate impulse scalar
            j = -(1 + self.e) * contactVel
            j /= invMassSum
            j /= self.contact_count

            # Apply impulse
            impulse = self.normal * j
            self.A.ApplyImpulse(-impulse, ra)
            self.B.ApplyImpulse(impulse, rb)

            # Friction impulse
            rv = (
                self.B.velocity
                + Cross(self.B.angularVelocity, rb)
                - self.A.velocity
                - Cross(self.A.angularVelocity, ra)
            )

            t = rv - (self.normal * Dot(rv, self.normal))
            t.normalize()

            # j tangent magnitude
            jt = -Dot(rv, t)
            jt /= invMassSum
            jt /= self.contact_count

            # Don't apply tiny friction impulses
            if Equal(jt, 0.0):
                return

            # Coulumb's law
            if abs(jt) < j * self.sf:
                tangentImpulse = t * jt
            else:
                tangentImpulse = t * -j * self.df

            # Apply friction impulse
            self.A.ApplyImpulse(-tangentImpulse, ra)
            self.B.ApplyImpulse(tangentImpulse, rb)

    def PositionalCorrection(self):
        k_slop = 0.05
        percent = 0.4
        correction = (
            max(self.penetration - k_slop, 0.0)
            / (self.A.invMass + self.B.invMass)
            * self.normal
            * percent
        )
        self.A.position -= correction * self.A.invMass
        self.B.position += correction * self.B.invMass

    def InfiniteMassCorrection(self):
        self.A.velocity.Set(0, 0)
        self.B.velocity.Set(0, 0)
