from common.math import *
from physic_engine.body import *
from physic_engine.manifold import *


class World:
    def __init__(self, dt, iterations):
        self.debugMode = False
        self.dt = dt
        self.iterations = iterations

        self.bodies = []
        self.contacts = []

        self.gravityScale = 5
        self.gravity = Vec2(0, 9.81 * self.gravityScale)

        self.score = 0

    def get_bodies(self):
        return self.bodies

    def add(self, shape, x, y):
        body = Body(shape, x, y)
        self.bodies.append(body)

    def step(self):
        # Generate new collision info
        contacts = []
        for i in range(0, len(self.bodies)):
            A = self.bodies[i]

            for j in range(i + 1, len(self.bodies)):
                B = self.bodies[j]
                if A.invMass == 0 and B.invMass == 0:
                    continue

                m = Manifold(A, B)
                m.Solve()
                if m.contact_count:
                    contacts.append(m)

        # Integrate forces
        for i in range(0, len(self.bodies)):
            self.integrateForces(self.bodies[i], self.dt)

        # Initialize collisions
        for i in range(0, len(contacts)):
            contacts[i].Initialize(self.dt, self.gravity)

        # Solve collisions
        for j in range(0, self.iterations):
            for i in range(0, len(contacts)):
                contacts[i].ApplyImpulse()

        # Correct positions
        for i in range(0, len(contacts)):
            contacts[i].PositionalCorrection()

        # Clear all forces
        for i in range(0, len(self.bodies)):
            body = self.bodies[i]
            body.force.Set(0, 0)
            body.torque = 0

    def integrateForces(self, b: Body, dt: float) -> None:
        if b.invMass == 0.0:
            return

        b.velocity += (b.force * b.invMass + self.gravity) * (dt / 2.0)
        b.angularVelocity += b.torque * b.invInertia * (dt / 2.0)

    def IntegrateVelocity(self, b: Body, dt: float) -> None:
        if b.invMass == 0.0:
            return

        b.position += b.velocity * dt
        b.orientation += b.angularVelocity * dt
        b.setOrient(b.orientation)
        self.integrateForces(b, dt)
