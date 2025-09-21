import gc
import yaml
import asyncio
import os
from threading import Lock

# NVidia API imports
import carb
import omni.kit.app
from isaacsim.core.api.world import World
from isaacsim.core.utils.stage import clear_stage, create_new_stage_async, update_stage_async, create_new_stage
from isaacsim.core.utils.viewports import set_camera_view
import omni.isaac.nucleus as nucleus

# Pedestrian Simulator internal API
from pedestrian.simulator.params import DEFAULT_WORLD_SETTINGS, SIMULATION_ENVIRONMENTS


class PedestrianInterface:
    _instance = None
    _is_initialized = False

    # Lock for safe multi-threading
    _lock: Lock = Lock()

    def __init__(self):

        if PedestrianInterface._is_initialized:
            return

        PedestrianInterface._is_initialized = True

        # Initialize the world with defalut settings
        self._world_settings = DEFAULT_WORLD_SETTINGS
        self._world = None

    @property
    def world(self):
        """The current isaacsim.core.api.world World instance

        Returns:
            isaacsim.core.api.world: The world instance
        """
        return self._world

    def initialize_world(self):
        """Method that initializes the world object
        """
        self._world = World(**self._world_settings)

    def load_asset(self, usd_asset: str, stage_prefix: str):
        """
        Method that will attempt to load an asset into the current simulation world, given the USD asset path.

        Args:
            usd_asset (str): The path where the USD file describing the world is located.
            stage_prefix (str): The name the vehicle will present in the simulator when spawned.
        """

        # Try to check if there is already a prim with the same stage prefix in the stage
        if self._world.stage.GetPrimAtPath(stage_prefix):
            raise Exception("A primitive already exists at the specified path")

        # Create the stage primitive and load the usd into it
        prim = self._world.stage.DefinePrim(stage_prefix)
        success = prim.GetReferences().AddReference(usd_asset)

        if not success:
            raise Exception("The usd asset" + usd_asset + "is not load at stage path " + stage_prefix)

    def set_viewport_camera(self, camera_position, camera_target):
        """Sets the viewport camera to given position and makes it point to another target position.

        Args:
            camera_position (list): A list with [X, Y, Z] coordinates of the camera in ENU inertial frame.
            camera_target (list): A list with [X, Y, Z] coordinates of the target that the camera should point to in the ENU inertial frame.
        """
        # Set the camera view to a fixed value
        set_camera_view(eye=camera_position, target=camera_target)

    def set_world_settings(self, physics_dt=None, stage_units_in_meters=None, rendering_dt=None, device=None):
        """
        Set the current world settings to the pre-defined settings. TODO - finish the implementation of this method.
        For now these new setting will never override the default ones.
        """
        # Set the physics engine update rate
        if physics_dt is not None:
            self._world_settings["physics_dt"] = physics_dt

        # Set the units of the simulator to meters
        if stage_units_in_meters is not None:
            self._world_settings["stage_units_in_meters"] = stage_units_in_meters

        # Set the render engine update rate (might not be the same as the physics engine)
        if rendering_dt is not None:
            self._world_settings["rendering_dt"] = rendering_dt

        if device is not None:
            self._world_settings["device"] = device

    def __new__(cls):
        """Allocates the memory and creates the actual PegasusInterface object is not instance exists yet. Otherwise,
        returns the existing instance of the PegasusInterface class.

        Returns:
            VehicleManger: the single instance of the VehicleManager class
        """

        # Use a lock in here to make sure we do not have a race condition
        # when using multi-threading and creating the first instance of the Pegasus extension manager
        with cls._lock:
            if cls._instance is None:
                cls._instance = object.__new__(cls)

        return PedestrianInterface._instance

    def __del__(self):
        """Destructor for the object. Destroys the only existing instance of this class."""
        PedestrianInterface._instance = None
        PedestrianInterface._is_initialized = False
