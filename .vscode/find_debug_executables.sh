#!/usr/bin/env bash
find -type f -executable ! -name "*.*" ! -name "MANIFEST" -exec sh -c 'readelf -S "$0" 2>/dev/null | grep -q "debug_info" && echo "$0"' {} \;
