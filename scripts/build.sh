#!/bin/bash
# build script for drone-pathgen project

set -e  # exit on any error

echo "🚀 building drone-pathgen project..."

# create build directory if it doesn't exist
if [ ! -d "build" ]; then
    echo "📁 creating build directory..."
    mkdir build
fi

# configure and build
echo "🔧 configuring and building..."
cd build
cmake ..
make

echo "✅ build complete!"

# optionally run test if requested
if [ "$1" = "--test" ] || [ "$1" = "-t" ]; then
    echo "🧪 running python example..."
    cd ..
    uv run python src/example_usage.py
fi

echo "🎉 all done!" 