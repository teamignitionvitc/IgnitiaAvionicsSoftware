#!/usr/bin/env python3
"""
Ignitia CanSat Avionics Build Script
Builds firmware and generates UF2 file using CMake and Pico SDK
"""

import os
import sys
import subprocess
import shutil
import argparse
from pathlib import Path


class Colors:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    CYAN = '\033[96m'
    RESET = '\033[0m'
    BOLD = '\033[1m'


def log(msg, color=Colors.RESET):
    print(f"{color}{msg}{Colors.RESET}")


def log_step(msg):
    log(f"\n{'='*60}", Colors.CYAN)
    log(f"  {msg}", Colors.CYAN + Colors.BOLD)
    log(f"{'='*60}", Colors.CYAN)


def log_success(msg):
    log(f"✓ {msg}", Colors.GREEN)


def log_error(msg):
    log(f"✗ {msg}", Colors.RED)


def log_warning(msg):
    log(f"⚠ {msg}", Colors.YELLOW)


def find_executable(name, env_var=None):
    """Find executable in PATH or environment variable"""
    if env_var and os.environ.get(env_var):
        path = Path(os.environ[env_var])
        if path.exists():
            return str(path)
    
    # Windows-specific: check common install locations FIRST
    if sys.platform == 'win32':
        # ARM GCC common locations
        if 'arm-none-eabi' in name:
            arm_paths = [
                Path('C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi'),
                Path('C:/Program Files/Arm GNU Toolchain arm-none-eabi'),
                Path('C:/Program Files (x86)/GNU Arm Embedded Toolchain'),
                Path('C:/Program Files/GNU Arm Embedded Toolchain'),
                Path(os.environ.get('USERPROFILE', 'C:/Users/User')) / 'AppData/Local/Programs/arm-gnu-toolchain',
            ]
            for base in arm_paths:
                if base.exists():
                    for version_dir in base.iterdir():
                        exe = version_dir / 'bin' / f'{name}.exe'
                        if exe.exists():
                            return str(exe)
        
        # Ninja - avoid node-gyp's fake ninja
        if name == 'ninja':
            ninja_paths = [
                Path('C:/Program Files/Ninja'),
                Path('C:/Program Files (x86)/Ninja'),
                Path('C:/ninja'),
                Path(os.environ.get('USERPROFILE', 'C:/Users/User')) / 'scoop/shims',
                Path(os.environ.get('USERPROFILE', 'C:/Users/User')) / '.local/bin',
            ]
            for p in ninja_paths:
                exe = p / 'ninja.exe'
                if exe.exists():
                    return str(exe)
    
    # Check PATH (but filter out node-gyp for ninja)
    result = shutil.which(name)
    if result:
        # Avoid node-gyp's fake ninja
        if name == 'ninja' and 'node' in result.lower():
            pass  # Skip this one
        else:
            return result
    
    return None


def check_prerequisites():
    """Check if required tools are installed"""
    log_step("Checking Prerequisites")
    
    tools = {
        'cmake': ('cmake', '--version'),
        'arm-none-eabi-gcc': ('arm-none-eabi-gcc', '--version'),
    }
    
    # Ninja is optional - we can use NMake on Windows
    optional_tools = {
        'ninja': ('ninja', '--version'),
    }
    
    missing = []
    found = {}
    
    for name, (cmd, arg) in tools.items():
        exe = find_executable(cmd)
        if exe:
            try:
                result = subprocess.run([exe, arg], capture_output=True, text=True)
                version = result.stdout.split('\n')[0] if result.stdout else 'unknown'
                log_success(f"{name}: {version}")
                found[name] = exe
            except Exception as e:
                log_warning(f"{name}: found but error running: {e}")
                found[name] = exe
        else:
            log_error(f"{name}: NOT FOUND")
            missing.append(name)
    
    # Check optional tools
    for name, (cmd, arg) in optional_tools.items():
        exe = find_executable(cmd)
        if exe:
            try:
                result = subprocess.run([exe, arg], capture_output=True, text=True)
                version = result.stdout.split('\n')[0] if result.stdout else 'unknown'
                log_success(f"{name}: {version}")
                found[name] = exe
            except Exception:
                pass
        else:
            log_warning(f"{name}: not found (will use NMake instead)")
    
    # Set PICO_TOOLCHAIN_PATH based on where we found arm-none-eabi-gcc
    if 'arm-none-eabi-gcc' in found:
        gcc_path = Path(found['arm-none-eabi-gcc'])
        toolchain_bin = gcc_path.parent
        os.environ['PICO_TOOLCHAIN_PATH'] = str(toolchain_bin)
        log_success(f"PICO_TOOLCHAIN_PATH: {toolchain_bin}")
    
    # Check PICO_SDK_PATH - also check local pico-sdk folder
    pico_sdk = os.environ.get('PICO_SDK_PATH')
    script_dir = Path(__file__).parent.resolve()
    local_sdk = script_dir / 'pico-sdk'
    
    if local_sdk.exists() and (local_sdk / 'pico_sdk_init.cmake').exists():
        # Use local SDK
        os.environ['PICO_SDK_PATH'] = str(local_sdk)
        log_success(f"PICO_SDK_PATH: {local_sdk} (local)")
    elif pico_sdk and Path(pico_sdk).exists():
        log_success(f"PICO_SDK_PATH: {pico_sdk}")
    else:
        log_error("PICO_SDK_PATH not set and no local pico-sdk found")
        missing.append('PICO_SDK_PATH')
    
    if missing:
        log_error(f"\nMissing: {', '.join(missing)}")
        log_warning("\nInstall instructions:")
        log("  CMake:     winget install Kitware.CMake")
        log("  ARM GCC:   winget install Arm.GnuArmEmbeddedToolchain")
        log("  Pico SDK:  git clone https://github.com/raspberrypi/pico-sdk.git")
        return None
    
    return found


def clean_build(build_dir):
    """Clean build directory"""
    log_step("Cleaning Build Directory")
    
    if build_dir.exists():
        # On Windows, git files can be locked - use rmdir /s /q
        if sys.platform == 'win32':
            try:
                subprocess.run(['cmd', '/c', 'rmdir', '/s', '/q', str(build_dir)], 
                             capture_output=True, check=False)
            except Exception:
                pass
        
        # Try Python method as fallback
        if build_dir.exists():
            def on_rm_error(func, path, exc_info):
                # Handle read-only files on Windows
                import stat
                os.chmod(path, stat.S_IWRITE)
                func(path)
            
            try:
                shutil.rmtree(build_dir, onerror=on_rm_error)
            except Exception as e:
                log_warning(f"Could not fully clean: {e}")
                log_warning("Try manually: rmdir /s /q build")
        
        if not build_dir.exists():
            log_success(f"Removed {build_dir}")
    
    build_dir.mkdir(parents=True, exist_ok=True)
    log_success(f"Created {build_dir}")


def configure_cmake(project_dir, build_dir, tools, generator=None):
    """Run CMake configuration"""
    log_step("Configuring CMake")
    
    # Auto-select generator
    if generator is None:
        if 'ninja' in tools:
            generator = 'Ninja'
        elif sys.platform == 'win32':
            generator = 'MinGW Makefiles'
        else:
            generator = 'Unix Makefiles'
    
    log(f"Using generator: {generator}")
    
    cmake_args = [
        tools['cmake'],
        '-S', str(project_dir),
        '-B', str(build_dir),
        '-G', generator,
        '-DCMAKE_BUILD_TYPE=Release',
    ]
    
    # Add Ninja path if using Ninja
    if generator == 'Ninja' and 'ninja' in tools:
        cmake_args.append(f'-DCMAKE_MAKE_PROGRAM={tools["ninja"]}')
    
    log(f"Running: {' '.join(cmake_args)}")
    
    result = subprocess.run(
        cmake_args,
        cwd=str(project_dir),
        capture_output=False
    )
    
    if result.returncode != 0:
        log_error("CMake configuration failed!")
        return False
    
    log_success("CMake configuration complete")
    return True


def build_firmware(build_dir, tools):
    """Build the firmware"""
    log_step("Building Firmware")
    
    build_args = [
        tools['cmake'],
        '--build', str(build_dir),
        '--parallel'
    ]
    
    log(f"Running: {' '.join(build_args)}")
    
    result = subprocess.run(
        build_args,
        capture_output=False
    )
    
    if result.returncode != 0:
        log_error("Build failed!")
        return False
    
    log_success("Build complete")
    return True


def find_uf2(build_dir):
    """Find generated UF2 file"""
    uf2_files = list(build_dir.glob('*.uf2'))
    if uf2_files:
        return uf2_files[0]
    return None


def copy_to_device(uf2_file):
    """Copy UF2 to RP2040 if connected in bootloader mode"""
    log_step("Looking for RP2040 Device")
    
    if sys.platform == 'win32':
        # Check for RPI-RP2 drive
        import string
        for letter in string.ascii_uppercase:
            drive = Path(f"{letter}:")
            if drive.exists():
                info_file = drive / "INFO_UF2.TXT"
                if info_file.exists():
                    log_success(f"Found RP2040 at {drive}")
                    dest = drive / uf2_file.name
                    shutil.copy(uf2_file, dest)
                    log_success(f"Copied {uf2_file.name} to {dest}")
                    log_success("Device will reboot automatically!")
                    return True
    else:
        # Linux/Mac - check /media or /Volumes
        mount_points = [Path('/media'), Path('/Volumes'), Path('/mnt')]
        for mount in mount_points:
            if mount.exists():
                for drive in mount.iterdir():
                    info_file = drive / "INFO_UF2.TXT"
                    if info_file.exists():
                        log_success(f"Found RP2040 at {drive}")
                        dest = drive / uf2_file.name
                        shutil.copy(uf2_file, dest)
                        log_success(f"Copied to {dest}")
                        return True
    
    log_warning("RP2040 not found in bootloader mode")
    log("To flash manually:")
    log("  1. Hold BOOTSEL button on RP2040-Zero")
    log("  2. Connect USB cable")
    log("  3. Release BOOTSEL")
    log(f"  4. Copy {uf2_file} to the RPI-RP2 drive")
    return False


def main():
    parser = argparse.ArgumentParser(description='Build Ignitia CanSat Firmware')
    parser.add_argument('--clean', action='store_true', help='Clean build directory first')
    parser.add_argument('--flash', action='store_true', help='Flash to device if connected')
    parser.add_argument('--build-dir', default='build', help='Build directory name')
    parser.add_argument('--generator', default='Ninja', choices=['Ninja', 'Unix Makefiles', 'NMake Makefiles'],
                        help='CMake generator')
    args = parser.parse_args()
    
    log(f"\n{Colors.BOLD}╔════════════════════════════════════════╗{Colors.RESET}")
    log(f"{Colors.BOLD}║   Ignitia CanSat Avionics Build Tool   ║{Colors.RESET}")
    log(f"{Colors.BOLD}╚════════════════════════════════════════╝{Colors.RESET}")
    
    # Determine directories
    script_dir = Path(__file__).parent.resolve()
    project_dir = script_dir
    build_dir = project_dir / args.build_dir
    
    log(f"\nProject: {project_dir}")
    log(f"Build:   {build_dir}")
    
    # Check prerequisites
    tools = check_prerequisites()
    if not tools:
        sys.exit(1)
    
    # Clean if requested
    if args.clean or not build_dir.exists():
        clean_build(build_dir)
    
    # Configure
    if not (build_dir / 'CMakeCache.txt').exists():
        if not configure_cmake(project_dir, build_dir, tools, args.generator):
            sys.exit(1)
    else:
        log_success("Using existing CMake configuration (use --clean to reconfigure)")
    
    # Build
    if not build_firmware(build_dir, tools):
        sys.exit(1)
    
    # Find UF2
    uf2_file = find_uf2(build_dir)
    if uf2_file:
        log_step("Build Output")
        log_success(f"UF2 file: {uf2_file}")
        log(f"Size: {uf2_file.stat().st_size:,} bytes")
        
        # Also show ELF if present
        elf_file = build_dir / 'ignitia_avionics.elf'
        if elf_file.exists():
            log(f"ELF file: {elf_file} ({elf_file.stat().st_size:,} bytes)")
        
        # Flash if requested
        if args.flash:
            copy_to_device(uf2_file)
    else:
        log_error("UF2 file not found!")
        sys.exit(1)
    
    log(f"\n{Colors.GREEN}{Colors.BOLD}Build successful!{Colors.RESET}\n")


if __name__ == '__main__':
    main()
