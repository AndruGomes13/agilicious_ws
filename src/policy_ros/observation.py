from abc import ABC, abstractmethod
from dataclasses import dataclass, fields
import functools
import math
from typing import ClassVar, Optional, Tuple, Union, get_args, get_origin
from typing_extensions import Annotated, Self
import jax.numpy as jp
import jax

Shape = Tuple[int, ...]

@dataclass(frozen=True)
class StateHistory:
    """A class to hold the history of states in an observation pipeline."""
    drone_state_history: jp.ndarray
    action_history: jp.ndarray
    ball_state_history: jp.ndarray


def observation_dataclass(cls):
    """Decorator that makes the obs dataclasses and builds the __FIELDS__"""
    derived: list[tuple[str, Shape]] = []
    for f in fields(cls):
        anno = cls.__annotations__[f.name]
        if get_origin(anno) is Annotated:
            base, shape = get_args(anno)
            if base is jp.ndarray:
                derived.append((f.name, shape))

    cls.__FIELD_SPECS__ = tuple(derived)

    return cls

@dataclass(frozen=True)
class Observation(ABC):
    __FIELD_SPECS__: ClassVar[Tuple[Tuple[str, Tuple[Union[int, str], ...]]]] = ()
    __FIELDS__: ClassVar[Tuple[Tuple[str, Shape]]] = tuple()
    ACTION_HISTORY_LEN: ClassVar[int] = 1
    SENSOR_DATA_HISTORY_LEN: ClassVar[int] = 1
    ACTION_BUFFER_LEN: ClassVar[int] = 1
    __resolved__: ClassVar[bool] = False

    @classmethod
    @abstractmethod
    def get_observation(cls, state: StateHistory) -> Self: #TODO
        pass

    def to_array(self) -> jp.ndarray:
        """Flattens the observation into a 1D array using the static field specification."""
        if not self.__class__.__resolved__:
            raise RuntimeError("You must call .resolve_fields(...) before using to_array()")

        arrays = []
        for field_name, _ in self.__FIELDS__:
            arrays.append(getattr(self, field_name).ravel())
        return jp.concatenate(arrays, axis=0)

    @classmethod
    def from_array(cls, arr: jp.ndarray):
        """Rebuilds an observation from the 1D array using the static field specification."""
        if not cls.__resolved__:
            raise RuntimeError("You must call .resolve_fields(...) before using from_array()")

        out = {}
        idx = 0
        for field_name, shape in cls.__FIELDS__:
            size = math.prod(shape)  # Compute the total size for this field.
            out[field_name] = arr[idx : idx + size].reshape(shape)
            idx += size

        assert idx == arr.size, f"got {arr.size} but expected {idx}"
        return cls(**out)

    @classmethod
    def resolve_fields(cls, action_history_len: Optional[int] = None, sensor_data_history_len: Optional[int] = None, action_buffer_len :Optional[int] = None):
        if action_history_len is not None:
            cls.ACTION_HISTORY_LEN = action_history_len
        if sensor_data_history_len is not None:
            cls.SENSOR_DATA_HISTORY_LEN = sensor_data_history_len
        if action_buffer_len is not None:
            cls.ACTION_BUFFER_LEN = action_buffer_len

        resolved = []
        for name, spec in cls.__FIELD_SPECS__:
            # turn any string dims into ints
            shape = tuple(getattr(cls, dim) if isinstance(dim, str) else dim for dim in spec)
            resolved.append((name, shape))
        cls.__FIELDS__ = tuple(resolved)
        cls.__resolved__ = True

        # wrap get_observation to auto‐validate
        orig = cls.get_observation.__func__  # unwrap the classmethod

        @functools.wraps(orig)
        def _wrapped(cls, state):
            obs = orig(cls, state)
            cls._validate(obs)
            return obs

        cls.get_observation = classmethod(_wrapped)

    @classmethod
    def _validate(cls, obs: Self):
        if not cls.__resolved__:
            raise RuntimeError("Must call resolve_fields(...) before using get_observation")
        for name, expected in cls.__FIELDS__:
            actual = getattr(obs, name).shape
            assert actual == expected, f"{cls.__name__}.{name} shape mismatch: got {actual}, expected {expected}"

    @classmethod
    def generate_random(cls, key: jp.ndarray) -> Self:
        if not cls.__resolved__:
            raise RuntimeError("Must call resolve_fields(...) before using get_observation")

        keys = jax.random.split(key, len(cls.__FIELDS__))

        random_attr = {}
        for (field_name, shape), k in zip(cls.__FIELDS__, keys):
            random_attr[field_name] = jax.random.normal(k, shape)

        return cls(**random_attr)

    def __eq__(self, other: Self) -> bool:
        for field_name, shape in self.__FIELDS__:
            self_value: jp.ndarray = getattr(self, field_name)
            other_value: jp.ndarray = getattr(other, field_name)

            # Check shape
            if self_value.shape != other_value.shape:
                return False

            # Check type
            if self_value.dtype != other_value.dtype:
                return False

            # Check if equal
            if not jp.allclose(self_value, other_value):
                return False

        return True
