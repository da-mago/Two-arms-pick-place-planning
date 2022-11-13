# Project common configurability

class GlobalConfig:

    # grid_num_layer
    #   Range: 1-3
    #   Note: XY is derived from excel content
    # 

    # actions_mode
    ACTIONS_BASIC          = 0 # Basic
    ACTIONS_DIAGONAL_2D    = 1 # Basic + Diag 2D
    ACTIONS_DIAGONAL_2D_3D = 2 # Basic + Diag 2D + Diag 3D

    # collision_min_distance
    # Range: any (unit: mm)

    def __init__(self,
                 grid_num_layers,
                 actions_mode,
                 collision_min_distance):

        self.grid_num_layers = grid_num_layers
        self.actions_mode = actions_mode
        self.collision_min_distance = collision_min_distance



