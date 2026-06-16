import subprocess
import os

# Check if running on Windows or Linux
if os.name == 'nt': # Windows
    vcvars_path = r"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
    compile_cmd = f'call "{vcvars_path}" && cl /LD /O2 /D_USE_MATH_DEFINES /Iinclude src/adaptation_controller.c src/psmsl_depth_processor.c /Fe:libadaptive_controller.dll'
else: # Linux/Posix
    compile_cmd = 'gcc -shared -O2 -fPIC -Iinclude src/adaptation_controller.c src/psmsl_depth_processor.c -o libadaptive_controller.so -lm'

print(f"Running command: {compile_cmd}")
result = subprocess.run(compile_cmd, shell=True, capture_output=True, text=True)

print("STDOUT:")
print(result.stdout)
print("STDERR:")
print(result.stderr)

if result.returncode == 0:
    print("[SUCCESS] Compilation completed successfully.")
else:
    print(f"[FAILURE] Compilation failed with exit code {result.returncode}.")
    import sys
    sys.exit(result.returncode)
