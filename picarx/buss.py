from readerwriterlock import rwlock


class Bus:
    """Interface used for message passing between threads."""

    def __init__(self) -> None:
        """Create a new bus."""
        self.msg = None
        self.lock = rwlock.RWLockWriteD()

    def read(self):
        with self.lock.gen_rlock():
            msg = self.msg
        return msg

    def write(self, msg):
        with self.lock.gen_wlock():
            self.message = msg
