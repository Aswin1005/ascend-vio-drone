"""
Delete all local arena_nav files and pull the correct versions from Jetson.
"""
import os
import paramiko

HOST = "192.168.218.49"
USER = "isro"
PASS = "isro@123"

LOCAL_FILES_TO_DELETE = [
    r"d:\open-vins\elimination_round\arena_nav_loiter_jetson.py",
    r"d:\open-vins\elimination_round\arena_nav_loiter.py",
    r"d:\open-vins\elimination_round\arena_nav_guided.py",
    r"d:\open-vins\elimination_round\arena_nav_guided_jetson.py",
    r"d:\open-vins\arena_nav_loiter.py",
    r"d:\open-vins\arena_nav_guided.py",
]

SAVE_DIR = r"d:\open-vins\elimination_round"

# ── Step 1: Delete local files ─────────────────────────────────────────────
print("=" * 55)
print("STEP 1: Deleting local arena_nav files...")
print("=" * 55)
for f in LOCAL_FILES_TO_DELETE:
    if os.path.exists(f):
        os.remove(f)
        print(f"  DELETED: {f}")
    else:
        print(f"  SKIP (not found): {f}")

# ── Step 2: Connect to Jetson ──────────────────────────────────────────────
print("\n" + "=" * 55)
print("STEP 2: Connecting to Jetson...")
print("=" * 55)
client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(HOST, username=USER, password=PASS, timeout=10)
print("  Connected!")

# ── Step 3: Find arena_nav files on Jetson ─────────────────────────────────
print("\n" + "=" * 55)
print("STEP 3: Finding arena_nav files on Jetson...")
print("=" * 55)
_, stdout, _ = client.exec_command(
    "find ~ /home -name 'arena_nav*.py' 2>/dev/null | grep -v __pycache__"
)
found = [l.strip() for l in stdout.read().decode().splitlines() if l.strip()]
print("  Found on Jetson:")
for f in found:
    print(f"    {f}")

# ── Step 4: SCP the loiter and guided versions ─────────────────────────────
print("\n" + "=" * 55)
print("STEP 4: Downloading from Jetson...")
print("=" * 55)

sftp = client.open_sftp()

# Filter to loiter and guided (exclude duplicates — prefer shortest path)
loiter_files = sorted([f for f in found if "loiter" in f], key=len)
guided_files  = sorted([f for f in found if "guided" in f], key=len)

def download(remote_path, local_name):
    local_path = os.path.join(SAVE_DIR, local_name)
    sftp.get(remote_path, local_path)
    size = os.path.getsize(local_path)
    print(f"  SCP: {remote_path}")
    print(f"    -> {local_path}  ({size} bytes)")

if loiter_files:
    download(loiter_files[0], "arena_nav_loiter.py")
else:
    print("  WARNING: No arena_nav_loiter.py found on Jetson!")

if guided_files:
    download(guided_files[0], "arena_nav_guided.py")
else:
    print("  WARNING: No arena_nav_guided.py found on Jetson!")

sftp.close()
client.close()

# ── Step 5: Confirm ────────────────────────────────────────────────────────
print("\n" + "=" * 55)
print("STEP 5: Final local state of elimination_round/")
print("=" * 55)
for f in os.listdir(SAVE_DIR):
    fp = os.path.join(SAVE_DIR, f)
    size = os.path.getsize(fp)
    print(f"  {f}  ({size} bytes)")

print("\nDone!")
