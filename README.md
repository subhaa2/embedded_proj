# embedded_proj

## Prerequisites
- CMake 3.12 or higher
- ARM GCC toolchain (arm-none-eabi-gcc)
- Pico SDK installed

## Setup
1. Clone the repository
   ```bash
   git clone <your-repo-url>
   cd embedded_proj
   ```

2. Set `PICO_SDK_PATH` environment variable to your Pico SDK location
   - **Windows (PowerShell)**: `$env:PICO_SDK_PATH="C:\path\to\pico-sdk"`
   - **Windows (CMD)**: `set PICO_SDK_PATH=C:\path\to\pico-sdk`
   - **Linux/Mac**: `export PICO_SDK_PATH=/path/to/pico-sdk`

## Build Methods

### Option 1: Using Make (Linux/Mac/MinGW)
```bash
mkdir build
cd build
cmake ..
make
```

### Option 2: Using Ninja (Faster builds)
```bash
mkdir build
cd build
cmake -G Ninja ..
ninja
```

### Option 3: Using CMake --build (Cross-platform)
```bash
mkdir build
cd build
cmake ..
cmake --build .
```

### Option 4: Using Visual Studio (Windows)
```bash
mkdir build
cd build
cmake -G "Visual Studio 17 2022" -A ARM ..
```
Then open the generated `.sln` file in Visual Studio

### Option 5: Using VS Code
1. Install the CMake and CMake Tools extensions
2. Open the project folder in VS Code
3. Press `Ctrl+Shift+P` and select "CMake: Configure"
4. Press `Ctrl+Shift+P` and select "CMake: Build"

## Testing Individual Modules

### Build Your Module
```bash
# Clean previous builds
rm -rf build

# Build
mkdir build && cd build
cmake ..
make yoursensor_module    # Replace with your module name
```

### Flash to Pico
1. Hold the BOOTSEL button while connecting your Pico to USB
2. Copy `build/yoursensor_module.uf2` to the RPI-RP2 drive
3. The Pico will automatically reboot

## Flashing to Pico
1. Hold the BOOTSEL button while connecting your Pico to USB
2. Copy the generated `build/embedded_proj.uf2` file to the RPI-RP2 drive
3. The Pico will automatically reboot and run your program


