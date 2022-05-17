from rlbot.utils.game_state_util import (
    BallState,
    Physics,
    Vector3,
)


class Ball:
    def __init__(self):
        pass

    def set_state(self) -> BallState:
        # TODO: Use mathematical state
        """
        Sets the ball's state
        """
        ball_state = BallState(
            Physics(
                location=Vector3(0, 1500, 1000),
                velocity=Vector3(0, -750, 1000),
            )
        )
        return ball_state
