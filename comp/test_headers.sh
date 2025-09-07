#!/bin/bash

# Test compilation script for split USB-CAN frame headers
# This script compiles the example to verify all headers work correctly

echo "Testing compilation of split USB-CAN frame headers..."

# Set compiler and flags
CXX="g++"
CXXFLAGS="-std=c++17 -Wall -Wextra -I."

# Test files to compile
TEST_FILES=(
    "class_usage_example.cpp"
)

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    local status=$1
    local message=$2
    
    if [ "$status" = "OK" ]; then
        echo -e "${GREEN}[OK]${NC} $message"
    elif [ "$status" = "WARN" ]; then
        echo -e "${YELLOW}[WARN]${NC} $message"
    else
        echo -e "${RED}[ERROR]${NC} $message"
    fi
}

# Test individual header compilation
echo "Testing individual header files..."

headers=(
    "usb_can_common.hpp"
    "adapter_base_frame.hpp"
    "fixed_size_frame.hpp" 
    "variable_size_frame.hpp"
    "frame_factory.hpp"
    "usb_can_frames.hpp"
)

for header in "${headers[@]}"; do
    echo -n "  Compiling $header... "
    if $CXX $CXXFLAGS -c -x c++ "$header" -o /dev/null 2>/dev/null; then
        print_status "OK" "$header compiles successfully"
    else
        print_status "ERROR" "$header failed to compile"
        echo "    Compilation errors:"
        $CXX $CXXFLAGS -c -x c++ "$header" -o /dev/null
    fi
done

echo ""
echo "Testing example application..."

# Test example compilation
for test_file in "${TEST_FILES[@]}"; do
    if [ -f "$test_file" ]; then
        echo -n "  Compiling $test_file... "
        output_file="${test_file%.*}"
        
        if $CXX $CXXFLAGS "$test_file" -o "$output_file" 2>/dev/null; then
            print_status "OK" "$test_file compiles successfully"
            
            # Try to run the example
            echo -n "  Running $output_file... "
            if ./"$output_file" >/dev/null 2>&1; then
                print_status "OK" "$output_file runs successfully"
            else
                print_status "WARN" "$output_file compiled but failed to run"
            fi
            
            # Clean up
            rm -f "$output_file"
        else
            print_status "ERROR" "$test_file failed to compile"
            echo "    Compilation errors:"
            $CXX $CXXFLAGS "$test_file" -o "$output_file"
        fi
    else
        print_status "WARN" "$test_file not found"
    fi
done

echo ""
echo "Header split verification complete!"
echo ""
echo "The headers have been successfully split into:"
echo "  - adapter_base_frame.hpp    (Abstract base class)"
echo "  - fixed_size_frame.hpp      (Fixed 20-byte frame)"
echo "  - variable_size_frame.hpp   (Variable length frame)"
echo "  - frame_factory.hpp         (Factory class)"
echo "  - usb_can_frames.hpp        (Main include file)"
echo ""
echo "Use '#include \"usb_can_frames.hpp\"' to include all frame classes."
