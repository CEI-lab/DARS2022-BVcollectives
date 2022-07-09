import numpy as np

import utils
# import robots
import param_sweep

if __name__ == '__main__':

    sim_time, ss = utils.load_global_config("configs/global_config.yaml")

    ############################ angle influence ###############################################

    # variables = "angle_influence"
    # metrics = np.array(["dist", "energy"])
    # iterations = 20

    # try:
    #     param_sweep.sweep("configs/global_config.yaml", "love", metrics, variables, iterations, sim_time, ss)
    # except Exception as e:
    #     print("Exception occured:")
    #     print(e)

    # try:
    #     param_sweep.sweep("configs/global_config.yaml", "aggression", metrics, variables, iterations, sim_time, ss)
    # except Exception as e:
    #     print("Exception occured:")
    #     print(e)

    # try:
    #     param_sweep.sweep("configs/global_config.yaml", "fear", metrics, variables, iterations, sim_time, ss)
    # except Exception as e:
    #     print("Exception occured:")
    #     print(e)

    # try:
    #     param_sweep.sweep("configs/global_config.yaml", "curiosity", metrics, variables, iterations, sim_time, ss)
    # except Exception as e:
    #     print("Exception occured:")
    #     print(e)

    ############################ angle influence semi-omni ###############################################

    # variables = "angle_influence"
    # metrics = np.array(["dist", "energy"])
    # iterations = 20

    # try:
    #     param_sweep.sweep("configs/global_config.yaml", "love_lantern", metrics, variables, iterations, sim_time, ss)
    # except Exception as e:
    #     print("Exception occured:")
    #     print(e)

    # try:
    #     param_sweep.sweep("configs/global_config.yaml", "aggression_lantern", metrics, variables, iterations, sim_time, ss)
    # except Exception as e:
    #     print("Exception occured:")
    #     print(e)

    # try:
    #     param_sweep.sweep("configs/global_config.yaml", "fear_lantern", metrics, variables, iterations, sim_time, ss)
    # except Exception as e:
    #     print("Exception occured:")
    #     print(e)

    # try:
    #     param_sweep.sweep("configs/global_config.yaml", "curiosity_lantern", metrics, variables, iterations, sim_time, ss)
    # except Exception as e:
    #     print("Exception occured:")
    #     print(e)

    ############################## influence noise ##############################################

    # variables = "influence_noise"
    # configs = ["love","aggression","fear","curiosity"]
    # metrics = ["dist", "energy"]
    # iterations = 20

    # try:
    #     param_sweep.sweep_noise("configs/global_config.yaml", configs, metrics, variables, iterations, sim_time, ss)
    # except Exception as e:
    #     print("Exception occured:")
    #     print(e)

    # ######################3######## influence noise lantern #######################################

    variables = "influence_noise"
    configs = ["love_lantern","aggression_lantern","fear_lantern","curiosity_lantern"]
    metrics = ["dist", "energy"]
    iterations = 20

    try:
        param_sweep.sweep_noise("configs/global_config.yaml", configs, metrics, variables, iterations, sim_time, ss)
    except Exception as e:
        print("Exception occured:")
        print(e)
