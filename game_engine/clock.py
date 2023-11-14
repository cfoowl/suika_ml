import pygame as pg


class Clock:
    def __init__(self):
        self.m_start = 0
        self.m_stop = 0
        self.m_current = 0
        self.Start()

    def Start(self):
        self.m_start = pg.time.get_ticks()

    def Stop(self):
        self.m_stop = pg.time.get_ticks()

    def Elapsed(self):
        self.m_current = pg.time.get_ticks()
        return (self.m_current - self.m_start) / 1000.0

    def Difference(self):
        return (self.m_stop - self.m_start) / 1000.0

    def Current(self):
        self.m_current = pg.time.get_ticks()
        return self.m_current
