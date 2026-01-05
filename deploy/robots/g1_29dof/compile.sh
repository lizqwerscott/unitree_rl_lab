#!/bin/bash

# Usage function
usage() {
    echo "Usage: $0 [-c] [-d] [-h]"
    echo "  -c    clean last build"
    echo "  -h    display this help message"
    echo "  -d    debug compile"
}
debug=0
# Parse options
while getopts "cdh" option; do
    case $option in
        c)  # Execute clean
            echo "clean last build!"
	        rm -rf ./install
	        rm -rf ./log
	        rm -rf ./build
            ;;
        h)  # Display help
            usage
            exit 0
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            usage
            exit 1
            ;;
    esac
done

shift $((OPTIND - 1))

echo "Building..."

rm -rf ./compile_commands.json
mkdir -p ./build && cd build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON -DCMAKE_BUILD_TYPE=Release ../
make -j6
cd ../
ln -s ./build/compile_commands.json compile_commands.json

echo "Build complete!"

