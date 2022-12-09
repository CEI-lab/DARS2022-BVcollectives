import numpy as np

import utils
import param_sweep

iterations = 20

if __name__ == '__main__':

    gconfig = "configs/global_config.yaml"

    ############################ angle influence full directional ###############################################

    variables = "angle_influence"
    metrics = np.array(["nnd", "energy"])
    type = "dir_dir"

    #try:
    #    param_sweep.sweep(gconfig, "love", metrics, variables, type, iterations)
    #except Exception as e:
    #    print("Exception occured:")
    #    print(e)

    #try:
    #    param_sweep.sweep(gconfig, "aggression", metrics, variables, type, iterations)
    #except Exception as e:
    #    print("Exception occured:")
    #    print(e)

    #try:
    #    param_sweep.sweep(gconfig, "fear", metrics, variables, type, iterations)
    #except Exception as e:
    #    print("Exception occured:")
    #    print(e)

    #try:
    #    param_sweep.sweep(gconfig, "curiosity", metrics, variables, type, iterations)
    #except Exception as e:
    #    print("Exception occured:")
    #    print(e)

    ############################ angle influence semi-omni ###############################################

    # variables = "angle_influence"
    # metrics = np.array(["nnd", "energy"])
    # type = "dir_omni"

    # try:
    #      param_sweep.sweep(gconfig, "love", metrics, variables, type, iterations)
    # except Exception as e:
    #     print("Exception occured:")
    #     print(e)

    # try:
    #     param_sweep.sweep(gconfig, "aggression", metrics, variables, type, iterations)
    # except Exception as e:
    #     print("Exception occured:")
    #     print(e)

    # try:
    #     param_sweep.sweep(gconfig, "fear", metrics, variables, type, iterations)
    # except Exception as e:
    #     print("Exception occured:")
    #     print(e)

    # try:
    #     param_sweep.sweep(gconfig, "curiosity", metrics, variables, type, iterations)
    # except Exception as e:
    #     print("Exception occured:")
    #     print(e)

    ############################ angle influence full omni ###############################################

    # variables = "angle_influence"
    # metrics = np.array(["nnd", "energy"])
    # type = "omni_omni"

    # try:
    #      param_sweep.sweep(gconfig, "love", metrics, variables, type, iterations)
    # except Exception as e:
    #     print("Exception occured:")
    #     print(e)

    # try:
    #     param_sweep.sweep(gconfig, "aggression", metrics, variables, type, iterations)
    # except Exception as e:
    #     print("Exception occured:")
    #     print(e)

    # try:
    #     param_sweep.sweep(gconfig, "fear", metrics, variables, type, iterations)
    # except Exception as e:
    #     print("Exception occured:")
    #     print(e)

    # try:
    #     param_sweep.sweep(gconfig, "curiosity", metrics, variables, type, iterations)
    # except Exception as e:
    #     print("Exception occured:")
    #     print(e)

    ############################## noise full dir ##############################################

    variables = "noise"
    type = "dir_dir"
    configs = ["love","aggression","fear","curiosity"]
    metrics = ["nnd", "energy"]

    try:
        param_sweep.sweep_noise("configs/global_config.yaml", configs, metrics, variables, type, iterations)
    except Exception as e:
        print("Exception occured:")
        print(e)

    # ############################## influence noise dir-omni #######################################

    # variables = "noise"
    # type = "dir_omni"
    # configs = ["love","aggression","fear","curiosity"]
    # metrics = ["nnd", "energy"]

    # try:
    #     param_sweep.sweep_noise("configs/global_config.yaml", configs, metrics, variables, type, iterations)
    # except Exception as e:
    #     print("Exception occured:")
    #     print(e)

    # ############################## influence noise omni-omni #######################################

    # variables = "noise"
    # type = "omni_omni"
    # configs = ["love","aggression","fear","curiosity"]
    # metrics = ["nnd", "energy"]

    # try:
    #     param_sweep.sweep_noise("configs/global_config.yaml", configs, metrics, variables, type, iterations)
    # except Exception as e:
    #     print("Exception occured:")
    #     print(e)
