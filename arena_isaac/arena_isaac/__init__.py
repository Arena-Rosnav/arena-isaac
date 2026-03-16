import queue
import typing


run_after_tick_queue: queue.Queue[typing.Callable[[], typing.Any]] = queue.Queue()
