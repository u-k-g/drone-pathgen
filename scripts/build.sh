#!/bin/bash
# build script for drone-pathgen project

set -e  # exit on any error

echo "🚀 Building drone-pathgen project..."

# create build directory if it doesn't exist
if [ ! -d "build" ]; then
    echo "📁 Creating build directory..."
    mkdir build
fi

# configure and build
echo "🔧 Configuring and building..."
cd build
cmake ..
make

echo "✅ Build complete!"

# optionally run test if requested
if [ "$1" = "--test" ] || [ "$1" = "-t" ]; then
    echo "🧪 Running test and generating output..."
    ./test > ../output/enhanced_output.txt
    echo "📄 Test output saved to output/enhanced_output.txt"
    
    # optionally run visualization if requested
    if [ "$2" = "--visualize" ] || [ "$2" = "-v" ]; then
        echo "🎨 Starting visualization..."
        cd ..
        uv run python visualization/visualize_open3d.py
    fi
fi

echo "🎉 All done!" 