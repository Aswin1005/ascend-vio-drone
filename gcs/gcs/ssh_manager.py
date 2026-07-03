"""
ssh_manager.py  —  Paramiko SSH + SFTP connection manager for Ascend GCS
"""

import os
import stat
import threading
import time

import paramiko


class SSHManager:
    """
    Manages a persistent SSH connection to the Jetson Nano.
    Provides exec_command wrappers and SFTP file transfer utilities.
    """

    DEFAULT_HOST = "isro.local"
    DEFAULT_USER = "isro"
    DEFAULT_PASS = "isro@123"
    DEFAULT_PORT = 22
    CONNECT_TIMEOUT = 10

    def __init__(self):
        self._client: paramiko.SSHClient | None = None
        self._lock = threading.Lock()
        self.connected = False
        self.ros_env_prefix = ""
        self.username = self.DEFAULT_USER

    # ─────────────────────────────────────────────
    #  Connection
    # ─────────────────────────────────────────────
    def connect(self, host=DEFAULT_HOST, user=DEFAULT_USER,
                password=DEFAULT_PASS, port=DEFAULT_PORT):
        """
        Open SSH connection.  Raises on failure.
        Returns (True, "") on success or (False, error_msg).
        """
        try:
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            client.connect(
                hostname=host,
                port=port,
                username=user,
                password=password,
                timeout=self.CONNECT_TIMEOUT,
                banner_timeout=20,
                auth_timeout=15,
            )
            with self._lock:
                if self._client:
                    try:
                        self._client.close()
                    except Exception:
                        pass
                self._client = client
                self.connected = True
                self.username = user
            
            # Retrieve ROS environment variables from remote
            self._load_ros_env(client)
            
            return True, ""
        except Exception as e:
            self.connected = False
            return False, str(e)

    def _load_ros_env(self, client):
        self.ros_env_prefix = ""
        exports = []
        
        # Method 1: Try running an interactive shell to get full ROS environment
        try:
            stdin, stdout, stderr = client.exec_command("bash -ic 'env | grep ROS'", timeout=5)
            lines = stdout.read().decode(errors="replace").splitlines()
            for line in lines:
                line = line.strip()
                if line and "=" in line:
                    key, val = line.split("=", 1)
                    if key.startswith("ROS_"):
                        exports.append(f"export {key}={val}")
        except Exception:
            pass
            
        # Method 2: Fallback to scanning ~/.bashrc if Method 1 returned nothing
        if not exports:
            try:
                stdin, stdout, stderr = client.exec_command('grep -E "ROS_MASTER_URI|ROS_IP|ROS_HOSTNAME" ~/.bashrc', timeout=5)
                lines = stdout.read().decode(errors="replace").splitlines()
                for line in lines:
                    line = line.strip()
                    if line and not line.startswith('#') and '=' in line:
                        if not line.startswith('export '):
                            line = 'export ' + line
                        exports.append(line)
            except Exception:
                pass
                
        # Method 3: Fallback to echo in login shell
        if not exports:
            try:
                stdin, stdout, stderr = client.exec_command("bash -lc 'echo ROS_MASTER_URI=$ROS_MASTER_URI; echo ROS_IP=$ROS_IP; echo ROS_HOSTNAME=$ROS_HOSTNAME'", timeout=5)
                lines = stdout.read().decode(errors="replace").splitlines()
                for line in lines:
                    line = line.strip()
                    if line and "=" in line:
                        key, val = line.split("=", 1)
                        if val.strip():
                            exports.append(f"export {key}={val}")
            except Exception:
                pass

        # Method 4: Fallback to /etc/environment
        if not exports:
            try:
                stdin, stdout, stderr = client.exec_command('cat /etc/environment', timeout=5)
                lines = stdout.read().decode(errors="replace").splitlines()
                for line in lines:
                    line = line.strip()
                    if line and not line.startswith('#') and '=' in line:
                        key, val = line.split("=", 1)
                        if "ROS" in key:
                            exports.append(f"export {key}={val}")
            except Exception:
                pass
                
        if exports:
            self.ros_env_prefix = " && ".join(exports) + " && "
            
        print("="*60)
        print("DEBUG GCS: Detected remote ROS env prefix:")
        print(self.ros_env_prefix if self.ros_env_prefix else "NONE")
        print("="*60)

    def disconnect(self):
        with self._lock:
            if self._client:
                try:
                    self._client.close()
                except Exception:
                    pass
                self._client = None
            self.connected = False

    def is_alive(self) -> bool:
        with self._lock:
            if not self._client:
                return False
            try:
                transport = self._client.get_transport()
                return transport is not None and transport.is_active()
            except Exception:
                return False

    # ─────────────────────────────────────────────
    #  Command execution (blocking, short commands)
    # ─────────────────────────────────────────────
    def exec(self, cmd: str, timeout: int = 15) -> tuple[int, str, str]:
        """
        Run a command and wait for it to finish.
        Returns (exit_code, stdout, stderr).
        """
        with self._lock:
            if not self._client:
                return -1, "", "Not connected"
            stdin, stdout, stderr = self._client.exec_command(cmd, timeout=timeout)
        stdout_str = stdout.read().decode(errors="replace")
        stderr_str = stderr.read().decode(errors="replace")
        exit_code = stdout.channel.recv_exit_status()
        return exit_code, stdout_str, stderr_str

    def exec_get_stdout(self, cmd: str, timeout: int = 15) -> str:
        _, out, _ = self.exec(cmd, timeout)
        return out.strip()

    # ─────────────────────────────────────────────
    #  Streaming command (returns channel for reading)
    # ─────────────────────────────────────────────
    def open_streaming_channel(self, cmd: str, environment: dict | None = None):
        """
        Open a persistent channel to run a long-running command.
        Returns (channel, stdin_file) so caller can read stdout line-by-line
        and also write to stdin.
        """
        with self._lock:
            if not self._client:
                raise RuntimeError("Not connected")
            transport = self._client.get_transport()

        channel = transport.open_session()
        channel.set_combine_stderr(True)   # merge stderr → stdout
        channel.get_pty()                  # allocate pseudo-TTY (needed for some ROS nodes)
        channel.exec_command(cmd)
        return channel

    def open_raw_channel(self, cmd: str):
        """Open channel WITHOUT pty — for commands that don't need TTY."""
        with self._lock:
            if not self._client:
                raise RuntimeError("Not connected")
        stdin, stdout, stderr = self._client.exec_command(cmd, get_pty=False)
        return stdin, stdout, stderr

    # ─────────────────────────────────────────────
    #  SFTP File Transfer
    # ─────────────────────────────────────────────
    def sftp_clear_remote_dir(self, remote_dir: str, progress_cb=None):
        """Recursively remove all contents of remote_dir (but keep the dir itself)."""
        with self._lock:
            if not self._client:
                raise RuntimeError("Not connected")
            sftp = self._client.open_sftp()
        try:
            self._sftp_rmtree_contents(sftp, remote_dir, progress_cb)
        finally:
            sftp.close()

    def _sftp_rmtree_contents(self, sftp, remote_dir, progress_cb=None):
        try:
            entries = sftp.listdir_attr(remote_dir)
        except FileNotFoundError:
            return
        for entry in entries:
            remote_path = remote_dir.rstrip("/") + "/" + entry.filename
            if stat.S_ISDIR(entry.st_mode):
                self._sftp_rmtree_contents(sftp, remote_path, progress_cb)
                try:
                    sftp.rmdir(remote_path)
                except Exception:
                    pass
            else:
                sftp.remove(remote_path)
                if progress_cb:
                    progress_cb(f"Removed: {remote_path}")

    def sftp_upload_folders(
        self,
        local_folders: list[str],
        remote_dest: str,
        progress_cb=None,
        total_progress_cb=None,
    ):
        """
        Upload all files from local_folders into remote_dest.
        Recreates the folder structure under remote_dest.
        progress_cb(msg: str) — called for each file
        total_progress_cb(done: int, total: int) — called for progress bar
        """
        with self._lock:
            if not self._client:
                raise RuntimeError("Not connected")
            sftp = self._client.open_sftp()

        try:
            # Count total files first
            all_files = []
            for folder in local_folders:
                folder_name = os.path.basename(folder.rstrip("/"))
                for root, dirs, files in os.walk(folder):
                    for fname in files:
                        local_path = os.path.join(root, fname)
                        rel = os.path.relpath(local_path, os.path.dirname(folder))
                        remote_path = remote_dest.rstrip("/") + "/" + rel.replace(os.sep, "/")
                        all_files.append((local_path, remote_path))

            total = len(all_files)
            done = 0

            for local_path, remote_path in all_files:
                # Ensure remote parent dir exists
                remote_parent = remote_path.rsplit("/", 1)[0]
                self._sftp_makedirs(sftp, remote_parent)
                sftp.put(local_path, remote_path)
                done += 1
                if progress_cb:
                    progress_cb(f"[{done}/{total}] {os.path.basename(local_path)}")
                if total_progress_cb:
                    total_progress_cb(done, total)

        finally:
            sftp.close()

    def _sftp_makedirs(self, sftp, remote_dir):
        """Create remote directory tree (like mkdir -p)."""
        dirs = []
        d = remote_dir
        while True:
            try:
                sftp.stat(d)
                break
            except FileNotFoundError:
                dirs.append(d)
                d = d.rsplit("/", 1)[0]
                if not d or d == "/":
                    break
        for d in reversed(dirs):
            try:
                sftp.mkdir(d)
            except Exception:
                pass

    def sftp_list_remote_scripts(self, remote_dir: str) -> list[str]:
        """Return list of .py filenames in remote_dir."""
        code, out, err = self.exec(f"ls {remote_dir}/*.py 2>/dev/null")
        if not out.strip():
            return []
        names = []
        for line in out.strip().splitlines():
            line = line.strip()
            if line:
                names.append(os.path.basename(line))
        return sorted(names)

    def sftp_download_folder(self, remote_dir: str, local_dest: str, progress_cb=None):
        """
        Recursively download all files from remote_dir on Jetson to local_dest on GCS.
        """
        with self._lock:
            if not self._client:
                raise RuntimeError("Not connected")
            sftp = self._client.open_sftp()
        try:
            os.makedirs(local_dest, exist_ok=True)
            self._sftp_download_dir_recursive(sftp, remote_dir, local_dest, progress_cb)
        finally:
            sftp.close()

    def _sftp_download_dir_recursive(self, sftp, remote_dir, local_dir, progress_cb=None):
        try:
            entries = sftp.listdir_attr(remote_dir)
        except IOError:
            # remote dir does not exist or couldn't read
            return
        for entry in entries:
            remote_path = remote_dir.rstrip("/") + "/" + entry.filename
            local_path = os.path.join(local_dir, entry.filename)
            if stat.S_ISDIR(entry.st_mode):
                os.makedirs(local_path, exist_ok=True)
                self._sftp_download_dir_recursive(sftp, remote_path, local_path, progress_cb)
            else:
                if progress_cb:
                    progress_cb(f"Downloading: {entry.filename}")
                sftp.get(remote_path, local_path)
