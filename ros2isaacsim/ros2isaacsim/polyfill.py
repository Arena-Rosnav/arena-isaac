
# typing_extensions.NoDefault
# https://github.com/python/typing_extensions/blob/abaaafd98c1cc7e5baf098ec287a3d22cb339670/src/typing_extensions.py#L999
import typing_extensions


class NoDefaultType():
    """The type of the NoDefault singleton."""

    __slots__ = ()

    def __new__(cls):
        return globals().get("NoDefault") or object.__new__(cls)

    def __repr__(self):
        return "typing_extensions.NoDefault"

    def __reduce__(self):
        return "NoDefault"


typing_extensions.NoDefault = NoDefaultType()
del NoDefaultType
