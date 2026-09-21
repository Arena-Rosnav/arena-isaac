import os
import re


def sanitize_path_component(component: str) -> str:
    component = re.sub(r'[^a-zA-Z0-9_]', '_', component)
    if component and not re.match(r'^[a-zA-Z_]', component):
        return f'_{component}'
    return component


def world_path(*path: str) -> str:
    if len(path) == 1:
        path = tuple(path[0].split(os.sep))
    return os.path.join('/World', *filter(None, map(sanitize_path_component, path)))
