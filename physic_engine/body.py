from common.shape import *
import random


class Body:
    def __init__(self, shape, x, y):
        self.shape = shape

        self.position = Vec2(x, y)
        self.velocity = Vec2(0, 0)

        self.angularVelocity = 0.0
        self.torque = 0.0
        self.orientation = math.radians(random.randint(0, 360))

        self.force = Vec2(0, 0)

        self.inertia = 0.0
        self.invInertia = 0.0
        self.mass = 0.0
        self.invMass = 0.0

        self.staticFriction = 0.5
        self.dynamicFriction = 0.3
        self.restitution = 0.2

        self.shape.body = self
        self.shape.ComputeMass(1.0)

    def ApplyForce(self, force: Vec2) -> None:
        self.force += force

    def ApplyImpulse(self, impulse: Vec2, contactVector: Vec2) -> None:
        self.velocity += impulse * self.invMass
        self.angularVelocity += self.invInertia * Cross(contactVector, impulse)

    def SetStatic(self) -> None:
        self.inertia = 0.0
        self.invInertia = 0.0
        self.mass = 0.0
        self.invMass = 0.0

    def SetOrient(self, radians: float) -> None:
        self.orientation = radians
        self.shape.SetOrientation(radians)
