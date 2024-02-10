# Project common configurability

class GlobalConfig:

    # grid_num_layer
    #   Range: 1-3
    #   Note: XY is derived from excel content
    # 

    # actions_mode
    ACTIONS_ORTHO_2D         = 0 # Orthogonal 2D
    ACTIONS_ORTHO_2D_DIAG_2D = 1 # Orthogonal 2D + Diagonal 2D
    ACTIONS_ORTHO_3D_DIAG_2D = 2 # Orthogonal 3D + Diagonal 2D
    ACTIONS_ORTHO_3D_DIAG_3D = 3 # Orthogonal 3D + Diagonal 3D

    # collision_min_distance
    # Range: any (unit: mm)

    def __init__(self,
                 grid_num_layers,
                 actions_mode,
                 collision_min_distance,
                 algorithm):

        self.grid_num_layers = grid_num_layers
        self.actions_mode = actions_mode
        self.collision_min_distance = collision_min_distance
        self.algorithm = algorithm



