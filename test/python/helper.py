from collections.abc import Iterator
from threading import Thread

import itertools

import rclpy
from rclpy.node import Node


def generate_arguments_list(
    # {A: {a, b}, B: {c, d}, ...}
    args: dict[str, set[str]],
) -> Iterator[tuple[str, str]]:
    """Generate launch arguments for the test."""
    # [[(A, a), (A, b)], [(B, c), (B, d)], ...]
    key_val_pairs = map(
        lambda key: itertools.product([key], args[key]),
        args.keys(),
    )
    # [((A, a), (B, c), ...), ((A, b), (B, c), ...), ..., ((A, a), (B, d), ...), ...]
    params = itertools.product(*key_val_pairs)
    # if a value (e.g. a, b, ...) equals to '', the pair will be filtered out
    params_filtered = map(
        lambda pairs: tuple(filter(lambda pair: pair[1] != '', pairs)),
        params,
    )

    return params_filtered


def arguments_list_to_dict(
    args: Iterator[tuple[str, str]],
) -> dict[str, str]:
    """Convert an iterator of tuples to a dictionary."""
    return {key: value for key, value in args}


def arguments_list_to_xacro_args(
    args: Iterator[tuple[str, str]],
) -> str:
    """Convert an iterator of tuples to a list of command line arguments."""
    return ' '.join(f'{key}:=\'"{value}"\'' for key, value in args)


def display_arguments_list(
    args: Iterator[tuple[str, str]],
) -> str:
    """Display the arguments list in a readable format."""
    return ', '.join(f'{key}="{value}"' for key, value in args)


class HelperNode(Node):
    """A test node that can be used in tests."""

    def __init__(self, name: str = 'test_node'):
        super().__init__(name)

    def start(self):
        """Spin the node."""
        self._ros_spin_thread = Thread(target=rclpy.spin, args=(self,))
        self._ros_spin_thread.start()
