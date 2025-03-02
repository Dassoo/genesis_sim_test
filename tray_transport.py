import numpy as np
import os
os.environ['PYOPENGL_PLATFORM'] = 'glx'

import genesis as gs


def main():
    gs.init(backend=gs.cpu)  # engine

    ############################### SCENE ####################################

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            dt=0.01,
            substeps=6
        ),
        coupler_options=gs.options.CouplerOptions(rigid_pbd=True, ),
        rigid_options=gs.options.RigidOptions(enable_collision=True, ),
        pbd_options=gs.options.PBDOptions(
            particle_size=0.01,
            gravity=(0, 0, -9.8),
        ),
        viewer_options=gs.options.ViewerOptions(
            camera_fov=50,
            res=(1920, 1080),
            max_FPS=60,
        ),
        show_viewer=True,
    )

    ############################# ENTITIES ##################################

    # PLANE
    plane = scene.add_entity(
        morph=gs.morphs.Plane(
            pos=(0.0, 0.0, 0.0),
        ),
        surface=gs.surfaces.Aluminium(
            ior=10.0,
        )
    )

    # TABLE LEGS TRAY ROBOT
    table1 = scene.add_entity(
        morph=gs.morphs.Box(
            pos=(0.0, 0.0, 0.2425),
            size=(1.2, 0.6, 0.485),
            euler=(0.0, 0.0, 90.0),
            fixed=True
        ),
        surface=gs.surfaces.Aluminium(
            color=(0.2, 0.2, 0.2),
        )
    )

    # TABLE LEGS SCANNER ROBOT
    table2 = scene.add_entity(
        morph=gs.morphs.Box(
            pos=(-0.9, 0.0, 0.2425),
            size=(1.2, 0.6, 0.485),
            euler=(0.0, 0.0, 0.0),
            fixed=True
        ),
        surface=gs.surfaces.Aluminium(
            color=(0.2, 0.2, 0.2),
        )
    )

    # TRAY TABLE
    table3 = scene.add_entity(
        morph=gs.morphs.Box(
            pos=(1.1, 0.0, 0.5),
            size=(0.3, 0.3, 1.0),
            euler=(0.0, 0.0, 90.0),
            fixed=True
        ),
        surface=gs.surfaces.Aluminium(
            color=(0.2, 0.2, 0.2),
        )
    )

    # TRAY
    tray = scene.add_entity(
        morph=gs.morphs.Box(
            pos=(1.0, 0.0, 1.005),
            size=(0.3, 0.6, 0.01),
            euler=(0.0, 0.0, 0.0),
            # fixed=True
        )
    )

    # TRAY ROBOT
    franka1 = scene.add_entity(
        morph=gs.morphs.MJCF(
            file='xml/franka_emika_panda/panda.xml',
            pos=(0.05, 0.0, 0.505)
        ),
        material=gs.materials.Rigid(coup_friction=4.0)
    )

    # SCANNER ROBOT
    franka2 = scene.add_entity(
        morph=gs.morphs.MJCF(
            file='xml/franka_emika_panda/panda.xml',
            pos=(-1.1, 0.0, 0.505)
        ),
        material=gs.materials.Rigid(coup_friction=4.0)
    )

    # TABLE TRAY ROBOT
    scene.add_entity(
        # material = gs.materials.Rigid(),
        morph=gs.morphs.Mesh(
            file="assets/table.ply",
            scale=0.001,
            pos=(0, 0, 0.5),
            euler=(90.0, 0.0, 90.0),
            # collision = True,
            fixed=True
        ),
        surface=gs.surfaces.Copper(
            color=(0.0, 0.0, 0.0),
        )
    )

    # TABLE SCANNER ROBOT
    scene.add_entity(
        morph=gs.morphs.Mesh(
            file="assets/table.ply",
            scale=0.001,
            pos=(-0.9, 0, 0.5),
            euler=(90.0, 0.0, 0.0),
            # collision = True,
            fixed=True
        ),
        surface=gs.surfaces.Copper(
            color=(0.0, 0.0, 0.0),
        )
    )

    # OBJ ON TRAY (TEST)
    glad = scene.add_entity(
        morph=gs.morphs.Mesh(
            file='assets/glad.obj',
            pos=(1.00, 0.0, 1.14),
            euler=(102.0, 0.0, 90.0),
            scale=0.002,
            # fixed = True
        )
    )

    scene.build()

    ############################ EXECUTION #################################

    jnt_names = [
        'joint1',
        'joint2',
        'joint3',
        'joint4',
        'joint5',
        'joint6',
        'joint7',
        'finger_joint1',
        'finger_joint2',
    ]
    dofs_idx = [franka1.get_joint(name).dof_idx_local for name in jnt_names]

    motors_dof = np.arange(7)
    fingers_dof = np.arange(7, 9)

    # set control gains
    franka1.set_dofs_kp(
        np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]),
    )
    franka1.set_dofs_kv(
        np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]),
    )
    franka1.set_dofs_force_range(
        np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
        np.array([87, 87, 87, 87, 12, 12, 12, 100, 100]),
    )
    franka2.set_dofs_kp(
        np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]),
    )
    franka2.set_dofs_kv(
        np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]),
    )
    franka2.set_dofs_force_range(
        np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
        np.array([87, 87, 87, 87, 12, 12, 12, 100, 100]),
    )

    # MOTION PLANNING
    hand1 = franka1.get_link('hand')
    hand2 = franka2.get_link('hand')

    # robot 2 on default position
    franka2.control_dofs_position(
        np.radians(np.array([0,0,0,0,0,0,0])),
        motors_dof,
    )

    # positioning pose
    qpos = franka1.inverse_kinematics(
        link=hand1,
        pos=np.array([0.65, 0.0, 1.01]),
        quat = np.array([0.5, 0.5, 0.5, 0.5])
    )

    # gripper open
    qpos[-2:] = 0.04

    path = franka1.plan_path(
        qpos_goal=qpos,
        num_waypoints=200,  # 2s duration
    )
    for waypoint in path:
        franka1.control_dofs_position(waypoint)
        scene.step()
    for i in range(100):
        scene.step()

    # move to pre-grasp pose
    qpos = franka1.inverse_kinematics(
        link=hand1,
        pos=np.array([0.77, 0.0, 1.01]),
        quat = np.array([0.5, 0.5, 0.5, 0.5])

    )
    path = franka1.plan_path(
        qpos_goal=qpos,
        num_waypoints=200,
    )
    for waypoint in path:
        franka1.control_dofs_position(waypoint)
        scene.step()
    for i in range(100):
        scene.step()

    # grasp
    franka1.control_dofs_position(qpos[:-2], motors_dof)
    franka1.control_dofs_force(np.array([-1, -1]), fingers_dof)
    for i in range(100):
        scene.step()
    franka1.control_dofs_force(np.array([-100, -100]), fingers_dof)
    for i in range(100):
        scene.step()

    # move tray up
    qpos = franka1.inverse_kinematics(
            link=hand1,
            pos=np.array([0.77, 0.0, 1.2]),
            quat=np.array([0.5, 0.5, 0.5, 0.5])

    )

    path = franka1.plan_path(
        qpos_goal=qpos,
        num_waypoints=200,
        ignore_collision=True,
        ignore_joint_limit=False
    )
    for waypoint in path:
        franka1.control_dofs_position(waypoint[:-2], motors_dof)
        scene.step()
    for i in range(200):
        scene.step()

    # move tray side
    qpos = franka1.inverse_kinematics(
            link=hand1,
            pos=np.array([0, 0.77, 1.2]),
            quat=np.array([0, 0, 0.7071, 0.7071])
    )

    path = franka1.plan_path(
        qpos_goal=qpos,
        num_waypoints=400,
        ignore_collision=True,
        ignore_joint_limit=False
    )
    for waypoint in path:
        franka1.control_dofs_position(waypoint[:-2], motors_dof)
        scene.step()
    for i in range(400):
        scene.step()

    # move tray back
    qpos = franka1.inverse_kinematics(
            link=hand1,
            pos=np.array([-0.35, 0, 1.2]),
            quat = np.array([-0.5, -0.5, 0.4, 0.4])
    )

    path = franka1.plan_path(
        qpos_goal=qpos,
        num_waypoints=400,
        ignore_collision=True,
        ignore_joint_limit=False
    )
    for waypoint in path:
        franka1.control_dofs_position(waypoint[:-2], motors_dof)
        scene.step()
    for i in range(400):
        scene.step()

    # move tray more back
    qpos = franka1.inverse_kinematics(
        link=hand1,
        pos=np.array([-0.2, 0, 1.2]),
        quat=np.array([-0.5, -0.5, 0.4, 0.4])
    )

    path = franka1.plan_path(
        qpos_goal=qpos,
        num_waypoints=200,
        ignore_collision=True,
        ignore_joint_limit=False
    )
    for waypoint in path:
        franka1.control_dofs_position(waypoint[:-2], motors_dof)
        scene.step()
    for i in range(200):
        scene.step()


    # back pose r2
    qpos = franka2.inverse_kinematics(
        link=hand2,
        pos=np.array([-1.1, 0.0, 1.3]),
        quat = np.array([0.5, 0.5, 0.5, 0.5])
    )

    # gripper open
    qpos[-2:] = 0.04

    path = franka2.plan_path(
        qpos_goal=qpos,
        num_waypoints=200,
    )
    for waypoint in path:
        franka2.control_dofs_position(waypoint)
        scene.step()
    for i in range(200):
        scene.step()


    # side pose r2 1
    qpos = franka2.inverse_kinematics(
        link=hand2,
        pos=np.array([-1.1, 0.4, 1.3]),
        quat = np.array([0.5, 0.5, 0.3, 0.3])
    )

    path = franka2.plan_path(
        qpos_goal=qpos,
        num_waypoints=200,
    )
    for waypoint in path:
        franka2.control_dofs_position(waypoint[:-2], motors_dof)
        scene.step()
    for i in range(200):
        scene.step()


    # side pose r2 2
    qpos = franka2.inverse_kinematics(
        link=hand2,
        pos=np.array([-0.8, 0.6, 1.4]),
        quat = np.array([0.5, 0.5, 0.1, 0.1])
    )

    path = franka2.plan_path(
        qpos_goal=qpos,
        num_waypoints=200,
    )
    for waypoint in path:
        franka2.control_dofs_position(waypoint[:-2], motors_dof)
        scene.step()
    for i in range(200):
        scene.step()


if __name__ == "__main__":
    main()