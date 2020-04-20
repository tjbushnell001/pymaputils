import os
import sys

dirpath = os.path.dirname(os.path.realpath(__file__))
pkgs_path = os.path.join(dirpath, "..", "src")
sys.path.append(pkgs_path)
