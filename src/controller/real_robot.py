from controller.interfaces import Action, Controller, Observation


class RealRobotController(Controller):
    def __init__(
        self,
        host: str = "localhost",
        port: int = 1024,
        step_time: float = 0.01,
    ):
        self.host = host
        self.port = port
        self.step_time = step_time

    def reset(self, seed: int | None = None) -> Observation:
        raise NotImplementedError("Real robot reset requires hardware")

    def step(self, action: Action) -> tuple[Observation, float, bool, bool]:
        raise NotImplementedError("Real robot step requires hardware")

    def close(self):
        pass
