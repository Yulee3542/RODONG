import os
import sys

# Add the parent folder containing the nodes/modules (patch(20260602)) to the import path.
# (use an absolute path so it works even with spaces/parens in the folder name)
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
