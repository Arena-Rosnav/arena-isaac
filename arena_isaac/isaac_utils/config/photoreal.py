import copy
import typing

import attrs
import carb
import carb.events
import omni.kit.actions.core
from omni.kit.viewport.utility import get_active_viewport, get_active_viewport_window

DLSSMode = typing.Literal['Auto', 'Quality', 'Balanced', 'Performance']
LightingRig = typing.Literal['Lights Off', 'Camera Light', 'Stage Lights', 'Colored Lights', 'Default', 'Grey Studio']

_DLSS_MODES: dict[str, DLSSMode] = {'auto': 'Auto', 'quality': 'Quality', 'balanced': 'Balanced', 'performance': 'Performance'}
_LIGHTING_RIGS: dict[str, LightingRig] = {
    'lights_off': 'Lights Off',
    'camera_light': 'Camera Light',
    'stage_lights': 'Stage Lights',
    'colored_lights': 'Colored Lights',
    'default': 'Default',
    'grey_studio': 'Grey Studio',
}
_OVERLAYS = ('axis', 'grid', 'bbox')


@attrs.define
class RenderSettings:
    @attrs.define
    class ViewportRender:
        resolution: typing.Literal['dynamic'] | tuple[int, int] = (1280, 720)
        scale: float = 1.0

        def apply(self):
            widget = get_active_viewport_window().viewport_widget

            if self.resolution == 'dynamic':
                widget.fill_frame = True
            else:
                widget.set_resolution(self.resolution)
            get_active_viewport().resolution_scale = self.scale

    @attrs.define
    class ViewportDisplay:
        axis: bool = True
        grid: bool = True
        bbox: bool = True

        def apply(self):

            action_registry = omni.kit.actions.core.get_action_registry()
            viewport_api = get_active_viewport()

            # Viewport Display settings
            action_registry.get_action("omni.kit.viewport.actions", "toggle_grid_visibility").execute(viewport_api=viewport_api, visible=self.grid)
            action_registry.get_action("omni.kit.viewport.actions", "toggle_axis_visibility").execute(viewport_api=viewport_api, visible=self.axis)
            action_registry.get_action("omni.kit.viewport.actions", "toggle_bounding_box_visibility").execute(viewport_api=viewport_api, visible=self.bbox)

    @attrs.define
    class RayTracing:
        DLSS: DLSSMode = 'Auto'

        def apply(self):
            settings = carb.settings.get_settings()
            settings.set_int("/rtx/post/dlss/execMode", {'Auto': 3, 'Quality': 2, 'Balanced': 1, 'Performance': 0}[self.DLSS])

    @attrs.define
    class Lighting:
        preset: LightingRig = 'Stage Lights'

        def apply(self):
            action_registry = omni.kit.actions.core.get_action_registry()
            action_registry.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_rig").execute(self.preset)

    @attrs.define
    class PostProcessing:
        # TODO

        def apply(self):
            ...
            # TODO

    render: ViewportRender = attrs.field(factory=ViewportRender)
    lighting: Lighting = attrs.field(factory=Lighting)
    display: ViewportDisplay = attrs.field(factory=ViewportDisplay)
    ray_tracing: RayTracing = attrs.field(factory=RayTracing)
    post_processing: PostProcessing = attrs.field(factory=PostProcessing)

    def apply(self):

        # Viewport Display settings
        try:
            self.display.apply()
        except Exception as e:
            carb.log_error(f"[RenderSettings] Failed to apply viewport display settings: {e}")

        # Viewport Render settings
        try:
            self.render.apply()
        except Exception as e:
            carb.log_error(f"[RenderSettings] Failed to apply viewport render settings: {e}")

        # Ray Tracing settings
        try:
            self.ray_tracing.apply()
        except Exception as e:
            carb.log_error(f"[RenderSettings] Failed to apply ray tracing settings: {e}")

        # Lighting settings
        try:
            self.lighting.apply()
        except Exception as e:
            carb.log_error(f"[RenderSettings] Failed to apply lighting settings: {e}")

        # Post Processing settings
        try:
            self.post_processing.apply()
        except Exception as e:
            carb.log_error(f"[RenderSettings] Failed to apply post processing settings: {e}")

    def apply_delayed(self):
        app = omni.kit.app.get_app()
        update_stream = app.get_update_event_stream()

        def callback(event: carb.events.IEvent):
            self.apply()
            if subscription:
                subscription.unsubscribe()

        subscription = update_stream.create_subscription_to_pop(
            callback,
            name="DelayedRenderSettingsApply",
        )


PRESET_DEFAULT: RenderSettings = RenderSettings()

PRESET_PHOTOREAL: RenderSettings = RenderSettings(
    render=RenderSettings.ViewportRender(
        resolution='dynamic',
        scale=1.0,
    ),
    lighting=RenderSettings.Lighting(
        preset='Default',
    ),
    display=RenderSettings.ViewportDisplay(
        axis=False,
        grid=False,
        bbox=False,
    ),
    ray_tracing=RenderSettings.RayTracing(
        DLSS='Quality',
    ),
    post_processing=RenderSettings.PostProcessing(),
)

PRESETS: dict[str, RenderSettings] = {
    'photoreal': PRESET_PHOTOREAL,
    'boring': PRESET_DEFAULT,
}


def _choice(key: str, value: str, allowed: typing.Iterable[str]) -> str:
    allowed = tuple(allowed)
    if value.lower() not in allowed:
        raise ValueError(f"viewport.{key}: expected one of {', '.join(allowed)}, got {value!r}")
    return value.lower()


def _resolution(value: str) -> typing.Literal['dynamic'] | tuple[int, int]:
    if value.lower() == 'dynamic':
        return 'dynamic'
    w, sep, h = value.lower().partition('x')
    if not sep or not w.isdigit() or not h.isdigit():
        raise ValueError(f"viewport.resolution: expected WxH or dynamic, got {value!r}")
    return int(w), int(h)


def _overlays(value: str) -> dict[str, bool]:
    names = {n.strip() for n in value.lower().split(',') if n.strip()}
    if names == {'none'}:
        names = set()
    if unknown := names - set(_OVERLAYS):
        raise ValueError(f"viewport.overlays: expected any of {', '.join(_OVERLAYS)} or none, got {', '.join(sorted(unknown))}")
    return {n: n in names for n in _OVERLAYS}


def resolve(preset: str, *, resolution: str = '', scale: str = '', dlss: str = '', lighting: str = '', overlays: str = '') -> RenderSettings:
    """Preset selected by name, each non-empty override replaces one field."""
    settings: RenderSettings = copy.deepcopy(PRESETS[_choice('preset', preset, PRESETS)])
    if resolution:
        settings.render.resolution = _resolution(resolution)
    if scale:
        settings.render.scale = float(scale)
    if dlss:
        settings.ray_tracing.DLSS = _DLSS_MODES[_choice('dlss', dlss, _DLSS_MODES)]
    if lighting:
        settings.lighting.preset = _LIGHTING_RIGS[_choice('lighting', lighting, _LIGHTING_RIGS)]
    if overlays:
        settings.display = RenderSettings.ViewportDisplay(**_overlays(overlays))
    return settings
