import typing

import omni.graph.core as og


def physics_engine() -> str:
    """Active physics engine name, 'physx' or 'newton'."""
    from isaacsim.core.simulation_manager import SimulationManager

    return SimulationManager.get_active_physics_engine()


_rebuilders: dict[str, typing.Callable[[], object]] = {}


def register_rebuilder(graph_path: str, rebuild: typing.Callable[[], object]) -> None:
    """Register a closure recreating a graph whose nodes cache physx tensor views,
    prim deletions invalidate those and only a rebuild re-acquires them."""
    _rebuilders[graph_path] = rebuild


def rebuild_graphs() -> None:
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    for graph_path, rebuild in list(_rebuilders.items()):
        parent = graph_path.rsplit('/', 1)[0]
        if not stage.GetPrimAtPath(parent).IsValid():
            del _rebuilders[graph_path]
            continue
        if stage.GetPrimAtPath(graph_path).IsValid():
            stage.RemovePrim(graph_path)
        rebuild()


class _Node:
    def __init__(
        self,
        master: "Graph",
        name: str,
        type_: str,
        *,
        values: typing.Iterable[tuple[str, typing.Any]] | None = None,
        connections: typing.Iterable[tuple[str, "_Node", str]] | None = None,
    ) -> None:
        self._master: Graph = master
        self._name: str = name
        self._type: str = type_

        if values is not None:
            for value in values:
                self.attribute(*value)

        if connections is not None:
            for connection in connections:
                self.connect(*connection)

    @property
    def name(self) -> str:
        return self._name

    @property
    def path(self) -> str:
        return f"{self._master.path}/{self._name}"

    @property
    def type(self) -> str:
        return self._type

    def create_attribute(self, attribute: str, type_: str):
        if attribute.startswith('outputs:'):
            attr_name = attribute[len('outputs:') :]
            self._master.add_action(
                lambda: og.Controller.create_attribute(
                    self.path,
                    attr_name,
                    type_,
                    og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT,
                )
            )
        else:
            self._master.add_action(lambda: og.Controller.create_attribute(self.path, attribute, type_))

    def attribute(self, input_: str, value: object):
        self._master.add_action(lambda: og.Controller.attribute(f"{self.path}.inputs:{input_}").set(value))

    def connect(self, output: str, node: "_Node", input_: str, *, outputs_prefix: str = 'outputs:'):
        self._master.add_action(lambda: og.Controller.connect(f"{self.path}.{outputs_prefix}{output}", f"{node.path}.inputs:{input_}"))


class Graph:
    def __init__(self, path: str) -> None:
        self._path: str = path
        self._nodes: list[_Node] = []
        self._actions: list[typing.Callable[[], typing.Any]] = []

    @property
    def path(self) -> str:
        return self._path

    def node(self, name: str, type_: str, **kwargs: object) -> _Node:
        self._nodes.append(node := _Node(self, name, type_, **kwargs))
        return node

    def add_action(self, action: typing.Callable[[], typing.Any]):
        self._actions.append(action)

    def execute_unsafe(self, controller: og.Controller) -> bool:
        controller.edit(
            {"graph_path": self.path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [(node.name, node.type) for node in self._nodes],
            },
        )

        for action in self._actions:
            action()

        return True

    def execute(self, controller: og.Controller) -> bool:
        try:
            self.execute_unsafe(controller)
        except Exception as e:
            raise e
            return False
        return True

    def load_extensions(self):
        from isaacsim.core.utils import extensions

        extensions.enable_extension("omni.graph.nodes")

        for node in self._nodes:
            ext_name = '.'.join(node.type.split('.')[:-1])
            extensions.enable_extension(ext_name)
