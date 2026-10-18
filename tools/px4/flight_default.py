"""The x500 reflex flight default (owner, 2026-10-18): candidate v5, the station
cascade compiled into ternary weights in frame v5 (832 bits), no learning, so it
flies with plasticity off. The trained prior (models/prior_x500.arc002, 480 bits)
is the learning demonstration. docs/px4_sitl_method.md, docs/gap_register.md G18.
No imports beyond the standard library."""
import os

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
BRAIN = os.path.join(REPO, 'models', 'prior_x500_candidate_v5.arc002')
BITS = 832
PLASTICITY = 'off'
LEARNING_DEMO = os.path.join(REPO, 'models', 'prior_x500.arc002')   # 480 bits, plasticity measured


def brain_input_bits(path):
    """Input width from an ARC002 header (u32 little-endian at byte 16)."""
    with open(path, 'rb') as f:
        h = f.read(20)
    return int.from_bytes(h[16:20], 'little')
