--[[
--  hb50w lua model for use with the RBDL library
--]]

-- Inertias
local base_i = {
    { 0.04644, 0.,      0.      },
    { 0.,      0.10130, 0.      },
    { 0.,      0.,      0.13120 }
}
local l0f_i = {
    { 0.001492, 0.,       0.       },
    { 0.,       0.001701, 0.       },
    { 0.,       0.,       0.001759 }
}
local l0r_i = {
    { 0.001474, 0.,       0.       },
    { 0.,       0.001666, 0.       },
    { 0.,       0.,       0.001711 }
}
local l1f_i = {
    { 0.006461, 0.,       0.       },
    { 0.,       0.002438, 0.       },
    { 0.,       0.,       0.007101 }
}
local l1r_i = {
    { 0.006329, 0.,       0.       },
    { 0.,       0.002338, 0.       },
    { 0.,       0.,       0.006986 }
}
local l2_i = {
    { 0.002319, 0.,        0.       },
    { 0.,       0.0001582, 0.       },
    { 0.,       0.,        0.002306 }
}

-- Mass parameters
local base = { mass = 9.222, com = { 0., 0., 0. }, inertia = base_i }
local l0f_m = 1.536
local l0r_m = 1.525
local l1f_m = 1.784
local l1r_m = 1.753
local l2_r = { mass = 0.526, com = { -0.001567, -0.190811, 0.000848 }, inertia = l2_i }
local l2_l = { mass = 0.526, com = { -0.001567, 0.190811, 0.000848 }, inertia = l2_i }

local joints = {
    -- Freeflyer joint created following the DoF order convention:
    -- x_rot, y_rot, z_rot, x_pos, y_pos, z_pos
    freeflyer = {
        { 1., 0., 0., 0., 0., 0. },
        { 0., 1., 0., 0., 0., 0. },
        { 0., 0., 1., 0., 0., 0. },
        { 0., 0., 0., 1., 0., 0. },
        { 0., 0., 0., 0., 1., 0. },
        { 0., 0., 0., 0., 0., 1. }
    },
    rot_x = {
        { 1., 0., 0., 0., 0., 0. }
    },
    fixed = {},
}

-- Transformations
local j0rot_rear = {
    { -1., -0., 0. },
    { 0.,  -1., 0. },
    { 0.,  0.,  1. }
}
-- J1 — FR, RL
local j1trans_fr_rl = { 0.0585, -0.0425, 0. }
local j1rot_fr_rl = {
    { 0., -1., 0. },
    { 1., 0.,  0. },
    { 0., 0.,  1. }
}
-- J1 — FL, RR
local j1trans_fl_rr = { 0.0585, 0.0425, 0. }
local j1rot_fl_rr = {
    { 0.,  1., 0. },
    { -1., 0., 0. },
    { 0.,  0., 1. }
}
-- J2 — right
local j2trans_r = { 0.10015, -0.22, 0. }
local j2rot_r = {
    { 1., 0.,        0.        },
    { 0., -0.939693, 0.342020  },
    { 0., -0.342020, -0.939693 }
}
-- J2 — left
local j2trans_l = { 0.10015, 0.22, 0. }
local j2rot_l = {
    { 1., 0.,        0.        },
    { 0., -0.939693, -0.342020 },
    { 0., 0.342020,  -0.939693 }
}
-- Feet — right
local foot_trans_r = { 0.0175, -0.22, 0. }
-- Feet — left
local foot_trans_l = { 0.0175, 0.22, 0. }

local bodies = {
    base = base,

    -- FR — leg 0
    fr_l0 = { 
        mass = l0f_m,
        com = { 0.059079, 0.004785, -0.000588 },
        inertia = l0f_i
    },
    fr_l1 = { 
        mass = l1f_m,
        com = { 0.055224, -0.015337, 0.003302 },
        inertia = l1f_i
    },
    fr_l2 = l2_r,

    -- FL
    fl_l0 = { 
        mass = l0f_m,
        com = { 0.059079, -0.004785, 0.000588 },
        inertia = l0f_i
    },
    fl_l1 = { 
        mass = l1f_m,
        com = { 0.055224, 0.015337, 0.003302 },
        inertia = l1f_i
    },
    fl_l2 = l2_l,

    -- RL
    rl_l0 = {
        mass = l0r_m,
        com = { 0.058684, 0.004487, 0.000600 },
        inertia = l0r_i
    },
    rl_l1 = { 
        mass = l1r_m,
        com = { 0.055591, 0.019865, 0.002679 },
        inertia = l1r_i
    },
    rl_l2 = l2_l,

    -- RR
    rr_l0 = {
        mass = l0r_m,
        com = { 0.058684, -0.004487, -0.000600 },
        inertia = l0r_i
    },
    rr_l1 = { 
        mass = l1r_m,
        com = { 0.055591, -0.019865, 0.002679 },
        inertia = l1r_i
    },
    rr_l2 = l2_r
}

-- Main model
local model = {
    gravity = { 0., 0., -9.81 },
    configuration = {
        axis_front = { 1., 0., 0.},
        axis_right = { 0., 1., 0.},
        axis_up    = { 0., 0., 1.}
    },

    frames = {
        {
            name = "base",
            parent = "ROOT",
            body = bodies.base,
            joint = joints.freeflyer
        },
        -- Leg 0 — FR
        {
            name = "fr_j0",
            parent = "base",
            body = bodies.fr_l0,
            joint = joints.rot_x,
            joint_frame = {
                r = { 0.1661, -0.064, 0. }
            }
        },
        {
            name = "fr_j1",
            parent = "fr_j0",
            body = bodies.fr_l1,
            joint = joints.rot_x,
            joint_frame = {
                r = j1trans_fr_rl,
                E = j1rot_fr_rl
            }
        },
        {
            name = "fr_j2",
            parent = "fr_j1",
            body = bodies.fr_l2,
            joint = joints.rot_x,
            joint_frame = {
                r = j2trans_r,
                E = j2rot_r
            }
        },
        {
            name = "fr_foot",
            parent = "fr_j2",
            joint = joints.fixed,
            joint_frame = {
                r = foot_trans_r
            }
        },
        -- Leg 1 — FL
        {
            name = "fl_j0",
            parent = "base",
            body = bodies.fl_l0,
            joint = joints.rot_x,
            joint_frame = {
                r = { 0.1661, 0.064, 0. }
            }
        },
        {
            name = "fl_j1",
            parent = "fl_j0",
            body = bodies.fl_l1,
            joint = joints.rot_x,
            joint_frame = {
                r = j1trans_fl_rr,
                E = j1rot_fl_rr
            }
        },
        {
            name = "fl_j2",
            parent = "fl_j1",
            body = bodies.fl_l2,
            joint = joints.rot_x,
            joint_frame = {
                r = j2trans_l,
                E = j2rot_l
            }
        },
        {
            name = "fl_foot",
            parent = "fl_j2",
            joint = joints.fixed,
            joint_frame = {
                r = foot_trans_l
            }
        },
        -- Leg 2 — RL
        {
            name = "rl_j0",
            parent = "base",
            body = bodies.rl_l0,
            joint = joints.rot_x,
            joint_frame = {
                r = { -0.1645, 0.064, 0. },
                E = j0rot_rear
            }
        },
        {
            name = "rl_j1",
            parent = "rl_j0",
            body = bodies.rl_l1,
            joint = joints.rot_x,
            joint_frame = {
                r = j1trans_fr_rl,
                E = j1rot_fr_rl
            }
        },
        {
            name = "rl_j2",
            parent = "rl_j1",
            body = bodies.rl_l2,
            joint = joints.rot_x,
            joint_frame = {
                r = j2trans_l,
                E = j2rot_l
            }
        },
        {
            name = "rl_foot",
            parent = "rl_j2",
            joint = joints.fixed,
            joint_frame = {
                r = foot_trans_l
            }
        },
        -- Leg 3 — RR
        {
            name = "rr_j0",
            parent = "base",
            body = bodies.rr_l0,
            joint = joints.rot_x,
            joint_frame = {
                r = { -0.1645, -0.064, 0. },
                E = j0rot_rear
            }
        },
        {
            name = "rr_j1",
            parent = "rr_j0",
            body = bodies.rr_l1,
            joint = joints.rot_x,
            joint_frame = {
                r = j1trans_fl_rr,
                E = j1rot_fl_rr
            }
        },
        {
            name = "rr_j2",
            parent = "rr_j1",
            body = bodies.rr_l2,
            joint = joints.rot_x,
            joint_frame = {
                r = j2trans_r,
                E = j2rot_r
            }
        },
        {
            name = "rr_foot",
            parent = "rr_j2",
            joint = joints.fixed,
            joint_frame = {
                r = foot_trans_r
            }
        }
    }
}

return model
