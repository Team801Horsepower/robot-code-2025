# co-authored by Ishaan Sen and ChatGPT, but it seems to work

import subprocess
import platform
import os

file_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "pipe.wav")

system = platform.system()

if system == "Windows":
    subprocess.run(
        ["powershell", "-c", f"Start-Process -FilePath {file_path}"], check=True
    )
elif system == "Darwin":  # macOS
    subprocess.run(["afplay", file_path], check=True)
elif system == "Linux":
    subprocess.run(["aplay", file_path], check=True)
