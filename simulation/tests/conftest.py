"""Puts simulation/ on the import path, as running a script from there does."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
