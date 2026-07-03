"""Push arena_nav_guided.py and arena_nav_loiter.py to Jetson scripts folder."""
import os, paramiko

HOST       = "isro.local"
USER       = "isro"
PASS       = "isro@123"
LOCAL_DIR  = r"d:\open-vins\elimination_round"
REMOTE_DIR = "/home/isro/catkin_ws/src/vio_bridge/scripts"

FILES = ["arena_nav_loiter.py", "arena_nav_guided.py"]

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
print(f"Connecting to {HOST}...")
client.connect(HOST, username=USER, password=PASS, timeout=10)
print("Connected!\n")

sftp = client.open_sftp()
for fname in FILES:
    local_path  = os.path.join(LOCAL_DIR, fname)
    remote_path = f"{REMOTE_DIR}/{fname}"
    sftp.put(local_path, remote_path)
    print(f"PUSHED {fname} ({os.path.getsize(local_path)} bytes) -> {remote_path}")

sftp.close()
client.close()
print("\nDone!")
