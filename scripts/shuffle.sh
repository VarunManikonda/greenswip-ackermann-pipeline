#!/bin/bash
# shuffle.sh - resets the world for the robustness demo
# Usage: ./shuffle.sh <1|2|3>

set -e
ARR=${1:-1}

reset_robot() {
    gz service -s /world/interview_world/set_pose \
        --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 \
        --req 'name: "ackermann_bot", position: {x: 0, y: 0, z: 0.05}, orientation: {x: 0, y: 0, z: 0, w: 1}' \
        > /dev/null
    echo "  Robot reset to origin facing +x"
}

place() {
    local name=$1; local x=$2; local y=$3; local z=$4
    gz service -s /world/interview_world/set_pose \
        --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 \
        --req "name: \"$name\", position: {x: $x, y: $y, z: $z}" \
        > /dev/null
    echo "  $name -> ($x, $y, $z)"
}

echo "============================================"
case $ARR in
  1)
    echo "Arrangement 1: DEFAULT (target on right)"
    echo "============================================"
    place target_box     3.0 -1.5 0.15
    place decoy_sphere   3.0 -0.7 0.20
    place decoy_capsule  3.0  0.0 0.25
    place decoy_cylinder 3.0  0.7 0.15
    ;;
  2)
    echo "Arrangement 2: BOX ON LEFT, decoys on right"
    echo "============================================"
    place target_box     3.0  1.5 0.15
    place decoy_sphere   2.5 -0.8 0.20
    place decoy_capsule  3.5  0.0 0.25
    place decoy_cylinder 2.5  0.5 0.15
    ;;
  3)
    echo "Arrangement 3: BOX FURTHER, decoys clustered"
    echo "============================================"
    place target_box     4.5  0.5 0.15
    place decoy_sphere   2.5  1.2 0.20
    place decoy_capsule  3.0 -1.0 0.25
    place decoy_cylinder 3.5 -1.5 0.15
    ;;
  *)
    echo "Unknown arrangement $ARR. Use 1, 2, or 3."
    exit 1
    ;;
esac

reset_robot
echo "============================================"
echo "Setup complete. Run control_node.py to chase."
