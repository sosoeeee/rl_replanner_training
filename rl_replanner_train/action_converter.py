import numpy as np
from typing import Dict, Tuple, Union
from gymnasium import spaces


class ActionConverter:
    """"
    Action class to store and standardize the action for the environment.
    """
    def __init__(self, action_setting: Dict):
        """"
        Initialization of an action converter.

        Args:
            action_space: The action space of the environment.
        """
        self.n = len(action_setting)
        self.parameters_min = []
        self.parameters_max = []
        for key in action_setting:
            _params_min = []
            _params_max = []
            _params_min.append([action_setting[key][k][0] for k in action_setting[key]])
            _params_max.append([action_setting[key][k][1] for k in action_setting[key]])
            self.parameters_min.append(np.array(_params_min).flatten())
            self.parameters_max.append(np.array(_params_max).flatten())
                   
    def gym_space_setting(self) -> spaces.Dict:
        """"
        Method to return the action space setting in Gym.

        Returns:
            The action space setting.
        """
        action_space_dic = {
            'id': spaces.Discrete(self.n),
        }

        for i in range(self.n):
            action_space_dic['params'+str(i)] = spaces.Box(self.parameters_min[i], self.parameters_max[i])  # avoid use "parameters" name to avoid conflict with the nn.DictModule

        action_space = spaces.Dict(action_space_dic)

        return action_space

    # get the exact type and parameters of the action from a concatenated action (Sapce.Dict)
    def convert(self, action: Dict[str, Union[int, np.ndarray]]) -> Tuple[int, list]:
        """"
        Method to convert the action from the concatenated form to the separated form.

        Args:
            action: The concatenated action.

        Returns:
            The separated action.
        """
        id_ = action['id']
        act_parameters_ = action['params'+str(id_)]
        return id_, act_parameters_