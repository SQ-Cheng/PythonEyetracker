from __future__ import annotations

import argparse
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path


def _ascii_preview(data: bytes) -> str:
    return "".join(chr(b) if 32 <= b < 127 else "." for b in data)


class TcpProbe:
    def __init__(self, listen_port: int, target_port: int, out_dir: Path):
        self.listen_port = listen_port
        self.target_port = target_port
        self.out_dir = out_dir
        self.ready = threading.Event()
        self.done = threading.Event()
        self.error: Exception | None = None
        self._server: socket.socket | None = None
        self._client: socket.socket | None = None
        self._target: socket.socket | None = None
        self._lock = threading.RLock()
        self._log = None
        self._bins: dict[str, object] = {}

    def start(self) -> threading.Thread:
        thread = threading.Thread(target=self.run, daemon=True)
        thread.start()
        return thread

    def run(self) -> None:
        self.out_dir.mkdir(parents=True, exist_ok=True)
        try:
            with (self.out_dir / "proxy.log").open("w", encoding="utf-8") as log:
                self._log = log
                self._bins["c2s"] = (self.out_dir / "client_to_server.bin").open("wb")
                self._bins["s2c"] = (self.out_dir / "server_to_client.bin").open("wb")
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                    self._server = server
                    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    server.bind(("127.0.0.1", self.listen_port))
                    server.listen(1)
                    self._write(f"listening 127.0.0.1:{self.listen_port} -> 127.0.0.1:{self.target_port}")
                    self.ready.set()
                    client, client_addr = server.accept()
                    with client:
                        self._client = client
                        self._write(f"accepted {client_addr}")
                        with socket.create_connection(("127.0.0.1", self.target_port), timeout=5.0) as target:
                            self._target = target
                            threads = [
                                threading.Thread(target=self._pump, args=(client, target, "c2s"), daemon=True),
                                threading.Thread(target=self._pump, args=(target, client, "s2c"), daemon=True),
                            ]
                            for thread in threads:
                                thread.start()
                            while not self.done.is_set() and any(thread.is_alive() for thread in threads):
                                time.sleep(0.05)
        except Exception as exc:
            self.error = exc
            self.ready.set()
            self._write(f"ERROR {exc!r}")
        finally:
            for sock in (self._client, self._target, self._server):
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass
            for file in self._bins.values():
                try:
                    file.close()
                except Exception:
                    pass
            self.done.set()

    def stop(self) -> None:
        self.done.set()
        for sock in (self._client, self._target, self._server):
            if sock is not None:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    sock.close()
                except OSError:
                    pass

    def _pump(self, src: socket.socket, dst: socket.socket, direction: str) -> None:
        total = 0
        count = 0
        while not self.done.is_set():
            try:
                data = src.recv(65536)
            except OSError:
                break
            if not data:
                break
            count += 1
            total += len(data)
            self._record(direction, count, total, data)
            try:
                dst.sendall(data)
            except OSError:
                break
        self._write(f"{direction} closed chunks={count} bytes={total}")

    def _record(self, direction: str, count: int, total: int, data: bytes) -> None:
        with self._lock:
            file = self._bins.get(direction)
            if file:
                file.write(data)
                file.flush()
            head = data[:96]
            self._write(
                f"{time.perf_counter():.6f} {direction} chunk={count} "
                f"len={len(data)} total={total} hex={head.hex(' ')} ascii={_ascii_preview(head)}"
            )

    def _write(self, message: str) -> None:
        with self._lock:
            if self._log is not None:
                self._log.write(message + "\n")
                self._log.flush()
            else:
                print(message, flush=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--listen-port", type=int, default=5779)
    parser.add_argument("--target-port", type=int, default=5777)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--max-frames", type=int, default=4)
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path(__file__).with_name("aseevr_proxy_probe"),
    )
    args = parser.parse_args()

    probe = TcpProbe(args.listen_port, args.target_port, args.out_dir)
    thread = probe.start()
    if not probe.ready.wait(timeout=3.0):
        print("proxy did not become ready", file=sys.stderr)
        return 2
    if probe.error:
        print(f"proxy failed: {probe.error!r}", file=sys.stderr)
        return 3

    test_script = Path(__file__).with_name("test_aseevr_eye_image_callback.py")
    command = [
        sys.executable,
        "-u",
        str(test_script),
        "--port",
        str(args.listen_port),
        "--timeout",
        str(args.timeout),
        "--max-frames",
        str(args.max_frames),
    ]
    try:
        ret = subprocess.run(command, check=False).returncode
    finally:
        probe.stop()
        thread.join(timeout=2.0)

    print(f"proxy artifacts: {args.out_dir}")
    return ret


if __name__ == "__main__":
    sys.exit(main())
