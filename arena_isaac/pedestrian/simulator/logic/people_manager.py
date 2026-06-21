import threading
import typing

import omni.kit.commands

from typing_extensions import Self


class PeopleManager:
    """The PeopleManager class is implemented following a singleton pattern. This means that once a person is spawned
    on the world or an instance of the PeopleManager is created, no either will be running at the same time.

    This class keeps track of all the people that are spawned in the simulation world, either trough the extension UI
    or via Python script. Every time a new person object is created, the 'add_person' method is invoked. Additionally,
    a person is removed, i.e. 'remove_person' gets invoked, every time the '__del__' function of the "Person" object
    gets invoked.
    """

    # The object instance of the people Manager
    _instance: typing.ClassVar[typing.Optional[Self]] = None
    _is_initialized = False

    # A dictionary of people that are spawned in the simulator
    _people = {}

    # Lock for safe multi-threading
    _lock: typing.ClassVar[threading.RLock] = threading.RLock()

    def __init__(self):
        """
        Constructor for the people manager class.
        """

        # __init__ re-runs on every get_people_manager() call, only configure once
        if PeopleManager._is_initialized:
            return
        PeopleManager._is_initialized = True

        # Rebuild the navigation mesh using the standard settings
        self.rebuild_nav_mesh()

    # Properties
    @property
    def people(self):
        """
        Returns:
            (list) List of people that were spawned.
        """
        return self._people

    # Operations
    @classmethod
    def get_people_manager(cls) -> Self:
        """
        Method that returns the current people manager.
        """
        return cls()

    def add_person(self, stage_prefix: str, person):
        """
        Method that adds the people to the person manager.

        Args:
            stage_prefix (str): A string with the name that the person is spawned in the simulator
            person (Person): The person object being added to the person manager.
        """
        with self._lock:
            self._people[stage_prefix] = person

    def get_person(self, stage_prefix: str):
        """Method that returns the person object given its stage prefix. Returns None if there is no person
        associated with that stage prefix

        Args:
            stage_prefix (str): A string with the name that the person is spawned in the simulator

        Returns:
            Person: The person object associated with the stage_prefix
        """
        with self._lock:
            return self._people.get(stage_prefix, None)

    def remove_person(self, stage_prefix: str):
        """
        Method that deletes a person from the person manager.

        Args:
            stage_prefix (str): A string with the name that the person is spawned in the simulator.
        """
        with self._lock:
            if stage_prefix in self._people:
                self._people.pop(stage_prefix).destroy()

    def remove_all_people(self):
        """
        Method that will delete all the people that were spawned from the people manager.
        """
        with self._lock:
            for person in list(self._people.values()):
                person.destroy()

    @classmethod
    def rebuild_nav_mesh(cls, height=1.5, radius=35.0, auto_rebake_on_changes=False, auto_rebake_delay_seconds=4, exclude_rigid_bodies=False, view_nav_mesh=False):
        """
        Rebuild the navmesh with the correct settings. Used for the people to move around.
        Called only when the sim with people is requested.
        """

        # Set the size of the people moving in the world
        omni.kit.commands.execute(
            'ChangeSetting',
            path='/exts/omni.anim.navigation.core/navMesh/config/agentHeight',
            value=height)
        omni.kit.commands.execute(
            'ChangeSetting',
            path='/exts/omni.anim.navigation.core/navMesh/config/agentRadius',
            value=radius)
        # Do not rebake the navigation mesh automatically
        omni.kit.commands.execute(
            'ChangeSetting',
            path='/persistent/exts/omni.anim.navigation.core/navMesh/autoRebakeOnChanges',
            value=auto_rebake_on_changes)
        omni.kit.commands.execute(
            'ChangeSetting',
            path='/persistent/exts/omni.anim.navigation.core/navMesh/autoRebakeDelaySeconds',
            value=auto_rebake_delay_seconds)
        # Exclude rigid bodies in the world, such as the drone
        omni.kit.commands.execute(
            'ChangeSetting',
            path='/exts/omni.anim.navigation.core/navMesh/config/excludeRigidBodies',
            value=exclude_rigid_bodies)
        # Do not show the navigation mesh
        omni.kit.commands.execute(
            'ChangeSetting',
            path='/persistent/exts/omni.anim.navigation.core/navMesh/viewNavMesh',
            value=view_nav_mesh)

    def __new__(cls):
        """Method that allocated memory for a new people_manager. Since the PeopleManager follows a singleton pattern,
        only one instance of PeopleManger object can be in memory at any time.

        Returns:
            PeopleManger: the single instance of the PeopleManager class.
        """

        # Use a lock in here to make sure we do not have a race condition
        # when using multi-threading and creating the first instance of the PeopleManager
        with cls._lock:
            if cls._instance is None:
                cls._instance = object.__new__(cls)
            return cls._instance

    def __del__(self):
        """Destructor for the object"""
        PeopleManager._instance = None
        return
