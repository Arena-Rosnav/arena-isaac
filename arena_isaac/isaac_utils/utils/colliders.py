from pxr import Usd, UsdPhysics


def uninstance_colliders(root: Usd.Prim) -> None:
    """Make every instanceable prim holding a collider under root a plain prim."""
    instances = [prim for prim in Usd.PrimRange(root) if prim.IsInstance()]
    for prim in instances:
        if any(item.HasAPI(UsdPhysics.CollisionAPI) for item in Usd.PrimRange(prim, Usd.TraverseInstanceProxies())):
            prim.SetInstanceable(False)
