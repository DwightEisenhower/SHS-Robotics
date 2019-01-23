#!/usr/bin/env python
# coding: utf-8

import tarfile
import logging
import os
import base64
import json

logger = logging.getLogger(__name__)

REQUIRED_FILES = ["main.cpp", "config.json", "robot-config.h"]
JSON_NAME = "___ThIsisATemPoRaRyFiLE___.json"


def read_file(file):
     with open(file, 'r') as f:
         return f.read()

def base64_file(file):
    return str(base64.b64encode(bytes(read_file(file), "utf-8")), "utf-8")


if __name__ == "__main__":  # Script
    # Argument parsing
    import argparse
    parser = argparse.ArgumentParser(description="Repack contents of unpacked VEX Coding Studio\'s binary project file "
                                                 "back into a component .vex file format. Allows for opening files"
                                                 "that were unpacked and editted externally.")
    parser.add_argument("folder", metavar="<folder_name>", help="Folder to repack.")
    parser.add_argument("--out_file", help="Output file path, defaults to \"<folder_name>.vex\"."
                                          "If exists, file WILL be overwritten.")
    parser.add_argument("--log_level", default="WARN", help="Set log level, one of WARN (default), INFO, DEBUG.")

    args = parser.parse_args()
    args.folder = args.folder.rstrip('/').rstrip('\\')

    if not args.out_file:
        args.out_file = os.path.split(args.folder)[-1] + ".vex"

    logging.basicConfig(level=args.log_level)

    # Error checking
    if not os.path.isdir(args.folder):
        raise ValueError("Directory '{}' is not a folder".format(args.folder))
    
    files = os.listdir(args.folder)
    for req_file in REQUIRED_FILES:
        if req_file not in files:
            raise ValueError("Directory '{}' is missing file '{}'".format(args.folder, req_file))
    
    # Repack
    json_str = read_file(args.folder + "/config.json")
    jdata = json.loads(json_str)
    files = {}

    for f in ["main.cpp", "robot-config.h"]:
        files[f] = base64_file(args.folder + "/" + f)
    jdata["files"] = files
    json_str = json.dumps(jdata)

    with open(JSON_NAME, "w") as f:
        f.write(json_str)

    with tarfile.open(args.out_file, "w") as tar:
        tar.add(JSON_NAME, arcname=JSON_NAME)

    os.remove(JSON_NAME)