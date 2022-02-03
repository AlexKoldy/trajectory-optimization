import math

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.game_state_util import (
    GameState,
    BallState,
    CarState,
    Physics,
    Vector3,
    Rotator,
    GameInfoState,
)


class Bot(BaseAgent):
    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        pass

    def initialize_agent(self):
        car_state = CarState(
            boost_amount=87,
            physics=Physics(
                velocity=Vector3(z=500),
                rotation=Rotator(math.pi / 2, 0, 0),
                angular_velocity=Vector3(0, 0, 0),
            ),
        )
        ball_state = BallState(Physics(location=Vector3(0, 0, None)))
        game_info_state = GameInfoState(world_gravity_z=700, game_speed=1)
        game_state = GameState(
            ball=ball_state, cars={self.index: car_state}, game_info=game_info_state
        )
        self.set_game_state(game_state)

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        pass
