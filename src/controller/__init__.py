from controller.interfaces import Action, Controller, Observation
from controller.simulator import SimulatorController
from controller.real_robot import RealRobotController


def make_controller(backend: str = "simulator", **kwargs) -> Controller:
    """Factory function to create a controller.
    
    Args:
        backend: "simulator" or "real"
        **kwargs: Configuration options passed to the controller
    
    Returns:
        Controller instance
    """
    if backend == "simulator":
        return SimulatorController(**kwargs)
    elif backend == "real":
        return RealRobotController(**kwargs)
    else:
        raise ValueError(f"Unknown backend: {backend}")
