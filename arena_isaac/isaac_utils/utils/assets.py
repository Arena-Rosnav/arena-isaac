import isaacsim.storage.native

_assets_root_path: str | None = None


def get_assets_root_path_safe(fallback: str = '/', *, reload: bool = False) -> str:
    """
    Try to get nucleus root path, fall back to `fallback` if not determinable.
    """
    global _assets_root_path
    if not reload and _assets_root_path is not None:
        return _assets_root_path

    try:
        root_path: str = isaacsim.storage.native.get_assets_root_path()
    except RuntimeError:
        root_path = fallback

    _assets_root_path = root_path
    return root_path
