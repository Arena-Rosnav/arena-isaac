"""Rebuild-on-dirty USD cache for pedestrian actors. stdlib + numpy + pxr.

convert_cached resolves an actor SDF to a per-actor cache dir holding the
authored USD (character.usda, clips/, meta.json), its downloaded source DAEs and
an ATTRIBUTION.md. The digest is deterministic offline: it hashes the SDF bytes
plus the referenced mesh URI strings, never the remote bytes. Importable outside
Isaac; pxr is only reached through peds.convert.convert_actor.
"""

from __future__ import annotations

import hashlib
import os
import pathlib
import shutil
import tempfile
import urllib.error
import urllib.request

from peds.convert import CONVERTER_VERSION, ActorSpec, convert_actor, parse_actor

_DOWNLOAD_RETRIES = 3
_DOWNLOAD_TIMEOUT = 60.0
_FALLBACK_DATA_DIR = pathlib.Path("/opt/arena_ws/data")


def _cache_root() -> pathlib.Path:
    env = os.environ.get("ARENA_DATA_DIR")
    if env:
        return pathlib.Path(env) / "peds_usd"
    if _FALLBACK_DATA_DIR.is_dir():
        return _FALLBACK_DATA_DIR / "peds_usd"
    return pathlib.Path(tempfile.gettempdir()) / "peds_usd"


def _digest(sdf_path: str, spec: ActorSpec) -> str:
    hasher = hashlib.sha256()
    hasher.update(str(CONVERTER_VERSION).encode("utf-8"))
    hasher.update(pathlib.Path(sdf_path).read_bytes())
    for uri in sorted(spec.mesh_uris):
        hasher.update(b"\n")
        hasher.update(uri.encode("utf-8"))
    return hasher.hexdigest()[:8]


def actor_cache_dir(sdf_path: str) -> pathlib.Path:
    """Deterministic cache dir for an actor SDF, without building anything."""
    spec = parse_actor(sdf_path)
    return _cache_root() / f"{spec.name}-{_digest(sdf_path, spec)}"


def convert_cached(sdf_path: str) -> pathlib.Path:
    """Return the cache dir for the actor, building it on a miss.

    On hit (character.usda and meta.json present) returns immediately. Otherwise
    builds into a tmp sibling and os.rename atomically into place.
    """
    spec = parse_actor(sdf_path)
    final = _cache_root() / f"{spec.name}-{_digest(sdf_path, spec)}"
    if (final / "character.usda").is_file() and (final / "meta.json").is_file():
        return final

    final.parent.mkdir(parents=True, exist_ok=True)
    tmp = pathlib.Path(tempfile.mkdtemp(prefix=f"{spec.name}-build-", dir=str(final.parent)))
    try:
        dae_paths = _fetch_all(spec, tmp, final)
        convert_actor(sdf_path, dae_paths, tmp)
        (tmp / "ATTRIBUTION.md").write_text(_attribution(spec))
        _publish(tmp, final)
    finally:
        if tmp.exists():
            shutil.rmtree(tmp, ignore_errors=True)
    return final


def _fetch_all(spec: ActorSpec, tmp: pathlib.Path, existing: pathlib.Path) -> dict[str, str]:
    paths: dict[str, str] = {}
    for index, uri in enumerate(spec.mesh_uris):
        dest = tmp / f"{index:02d}_{_uri_basename(uri)}"
        _fetch(uri, dest, existing)
        paths[uri] = str(dest)
    return paths


def _fetch(uri: str, dest: pathlib.Path, existing: pathlib.Path) -> None:
    prior = existing / dest.name
    if prior.is_file():
        shutil.copyfile(prior, dest)
        return
    if _is_remote(uri):
        _download(uri, dest)
        return
    source = uri[len("file://") :] if uri.startswith("file://") else uri
    shutil.copyfile(source, dest)


def _is_remote(uri: str) -> bool:
    return uri.startswith(("http://", "https://"))


def _download(uri: str, dest: pathlib.Path) -> None:
    last_error: Exception | None = None
    for _ in range(_DOWNLOAD_RETRIES):
        try:
            with urllib.request.urlopen(uri, timeout=_DOWNLOAD_TIMEOUT) as response, open(dest, "wb") as handle:
                shutil.copyfileobj(response, handle)
            return
        except (urllib.error.URLError, OSError) as error:
            last_error = error
    raise RuntimeError(f"failed to download {uri} after {_DOWNLOAD_RETRIES} attempts: {last_error}")


def _uri_basename(uri: str) -> str:
    name = uri.split("?")[0].rstrip("/").split("/")[-1]
    return name or "mesh.dae"


def _publish(tmp: pathlib.Path, final: pathlib.Path) -> None:
    try:
        os.rename(tmp, final)
    except OSError:
        # Another builder won the race, keep its result.
        if (final / "character.usda").is_file():
            return
        raise


def _attribution(spec: ActorSpec) -> str:
    sources = "\n".join(f"- {uri}" for uri in spec.mesh_uris)
    return (
        f"# Attribution\n\n"
        f"The skin and animation clips for actor `{spec.name}` are converted from\n"
        f"Gazebo Fuel assets:\n\n"
        f"{sources}\n\n"
        f"(c) Mingfei, distributed via Gazebo Fuel under CC-BY-4.0.\n"
        f"This USD conversion redistributes them under the same license.\n"
    )
