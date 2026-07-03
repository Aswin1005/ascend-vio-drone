"""
Pull arena_nav_loiter.py and arena_nav_guided.py from Jetson.
Run this when Jetson is reachable on the network.
"""
import os
import paramiko

HOST = "isro.local"
USER = "isro"
PASS = "isro@123"
SAVE_DIR = r"d:\open-vins\elimination_round"

print("Connecting to Jetson...")
client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(HOST, username=USER, password=PASS, timeout=10)
print("Connected!")

# Find files
_, stdout, _ = client.exec_command(
    "find ~ /home -name 'arena_nav*.py' 2>/dev/null | grep -v __pycache__"
)
found = [l.strip() for l in stdout.read().decode().splitlines() if l.strip()]
print("\nFound on Jetson:")
for f in found: print(f"  {f}")

sftp = client.open_sftp()

loiter_files = sorted([f for f in found if "loiter" in f], key=len)
guided_files  = sorted([f for f in found if "guided" in f], key=len)

def download(remote_path, local_name):
    local_path = os.path.join(SAVE_DIR, local_name)
    sftp.get(remote_path, local_path)
    print(f"\nSCP: {remote_path}\n  -> {local_path}  ({os.path.getsize(local_path)} bytes)")

if loiter_files:
    download(loiter_files[0], "arena_nav_loiter.py")
else:
    print("WARNING: No arena_nav_loiter.py found on Jetson!")

if guided_files:
    download(guided_files[0], "arena_nav_guided.py")
else:
    print("WARNING: No arena_nav_guided.py found on Jetson!")

sftp.close()
client.close()

print("\nFinal local state:")
for f in sorted(os.listdir(SAVE_DIR)):
    print(f"  {f}  ({os.path.getsize(os.path.join(SAVE_DIR, f))} bytes)")
print("\nDone!")
