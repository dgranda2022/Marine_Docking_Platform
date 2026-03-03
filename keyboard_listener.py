"""Non-blocking keyboard listener for real-time control loop parameter updates."""

import select
import sys
import termios
import threading
import tty


class KeyboardListener:
    """Threaded raw-mode keyboard listener that mutates a shared State Dictionary.

    Designed to run as a daemon sidecar alongside a high-frequency control
    loop. Reads single keypresses without blocking the main thread and
    writes state changes directly into the shared dictionary so the
    control loop picks them up on its next iteration.

    Attributes:
        state: Reference to the shared State Dictionary owned by the
            orchestrator. This listener mutates entries in-place.
        running: Flag that governs the listen loop. Set to False to
            request a clean shutdown.
        thread: The underlying threading.Thread instance. Created in
            __init__ but not started until start() is called.
    """

    def __init__(self, state_dict: dict) -> None:
        """Store the shared state reference and prepare the listener thread.

        Args:
            state_dict: The orchestrator's master State Dictionary. Must
                contain a 'Landing_Logic' key with an 'armed' boolean.
        """
        self.state: dict = state_dict
        self.running: bool = True
        self.thread: threading.Thread = threading.Thread(
            target=self.listen,
            daemon=True,
        )

    def _get_key(self) -> str:
        """Read a single character from stdin in raw terminal mode.

        Switches the terminal to raw mode for exactly one read, then
        restores the original settings immediately. This prevents the
        terminal from staying in a broken state if the program crashes
        or is interrupted between the mode switch and the restore.

        Returns:
            A single-character string representing the key pressed.
        """
        fd: int = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch: str = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def listen(self) -> None:
        """Poll stdin for keypresses and update the shared State Dictionary.

        This is the thread target. It loops until self.running is False,
        using select.select with a 100 ms timeout so the loop remains
        responsive to shutdown requests even when no keys are pressed.

        Key bindings:
            l / L — Toggle Landing_Logic armed state.
            q     — Request listener shutdown.
        """
        while self.running:
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not ready:
                continue

            key: str = self._get_key()

            if key in ("l", "L"):
                current: bool = self.state["Landing_Logic"]["armed"]
                self.state["Landing_Logic"]["armed"] = not current
                label: str = "ARMED" if not current else "DISARMED"
                print(f"\r[INPUT] Landing Logic {label}: {not current} ", end="", flush=True)

            elif key == "q":
                self.running = False

    def start(self) -> None:
        """Launch the listener on its daemon thread."""
        self.thread.start()

    def stop(self) -> None:
        """Signal the listener to stop and wait for the thread to exit."""
        self.running = False
        self.thread.join()