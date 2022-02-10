def get_integrator(dt, model, integrator="RK4"):
    """Factory for integrators: Euler, Heun, RK4, AB2"""
    integrators = dict(
        # Euler=Euler(dt, model),
        # Heun=Heun(dt, model),
        RK4=RungeKutta4(dt, model),
        # AB2=AdamsBashforth2(dt, model),
    )
    return integrators[integrator]


class Integrator:
    """Integrator for a system of first-order ordinary differential equations
    of the form \dot x = f(t, x, u).
    """

    def __init__(self, dt, f):
        self.dt = dt
        self.f = f

    def step(self, x, u):
        raise NotImplementedError


class RungeKutta4(Integrator):
    def step(self, x, u):
        k1 = self.f(x, u)
        k2 = self.f(x + 0.5 * self.dt * k1, u)
        k3 = self.f(x + 0.5 * self.dt * k2, u)
        k4 = self.f(x + self.dt * k3, u)
        return x + self.dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6
