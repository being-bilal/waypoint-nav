import threading
import queue

_queues: dict[str, queue.Queue] = {}
_lock   = threading.Lock()
_started = False


def _get_queue(msg_type: str) -> queue.Queue:
    with _lock:
        if msg_type not in _queues:
            _queues[msg_type] = queue.Queue(maxsize=20)
        return _queues[msg_type]


def start(master):
    """Call once from main.py with the shared mavlink connection."""
    global _started
    if _started:
        return
    _started = True

    def _run():
        while True:
            msg = master.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            if msg.get_type() == "BAD_DATA":
                continue

            q = _get_queue(msg.get_type())
            try:
                q.put_nowait(msg)
            except queue.Full:
                q.get_nowait()
                q.put_nowait(msg)

    t = threading.Thread(target=_run, daemon=True, name="mavlink-dispatcher")
    t.start()


def get_message(msg_type: str, timeout: float = 2.0):
    return _get_queue(msg_type).get(timeout=timeout)