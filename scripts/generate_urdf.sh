#!/bin/bash
# Regenerate plain URDFs from xacro. Run after editing any .xacro.
set -e
cd "$(dirname "$0")/.."
for r in hb50 hb50w hb40; do
    xacro urdf/$r/$r.urdf.xacro -o urdf/$r/$r.urdf
done
