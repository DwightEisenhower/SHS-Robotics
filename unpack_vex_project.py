#!/usr/bin/env python
# coding: utf-8
import json
import binascii
import tarfile
from pathlib import Path
import logging

logger = logging.getLogger(__name__)


def extract_vex_contents_dict(vex_file):
    with tarfile.open(vex_file) as tf:
        file_info, = tf.getmembers()
        fh = tf.extractfile(file_info)
        return json.load(fh)


def prepare_out_dir(out_dir):
    out_dir = Path(out_dir)
    if out_dir.exists():
        if not out_dir.is_dir():
            raise ValueError(f'Output destination "{out_dir}" must be a directory')
        for f in out_dir.iterdir():
            logger.info('Deleting %s', f)
            f.unlink()
            pass
    else:
        logger.info('Making directory %s', out_dir)
        out_dir.mkdir(parents=True)
        pass
    return out_dir


def write_output_files(out_dir, vex_dict, files_dict):
    fn = out_dir / 'config.json'
    with open(fn, 'xt', newline='\n', encoding='utf-8') as fp:
        json.dump(vex_dict, fp, indent=4, sort_keys=True)
        logger.info('Wrote %s', fn)
        pass
    for file_name, encoded_contents in files_dict.items():
        contents = binascii.a2b_base64(encoded_contents).decode("utf-8")
        fn = out_dir / file_name
        with open(fn, 'xt', newline='\n', encoding='utf-8') as fp:
            fp.write(contents)
            logger.info('Wrote %s', fn)
            pass
        pass
    return


def execute(vex_file, out_dir):
    vex_dict = extract_vex_contents_dict(vex_file)
    files_dict = vex_dict['files']
    vex_dict['files'] = list(sorted(files_dict.keys()))  # replace with the list of file names
    out_dir = prepare_out_dir(out_dir)
    write_output_files(out_dir, vex_dict, files_dict)
    return


if __name__ == '__main__':  # script
    import argparse
    parser = argparse.ArgumentParser(description='Unpack contents of the VEX Coding Studio\'s binary project file '
                                                 'and save its component source code files in a directory. '
                                                 'This is useful to simplify tracking of the source code changes.')
    parser.add_argument('vex_file', metavar='<project_name>.vex', help='VEX project file.')
    parser.add_argument('--out_dir', help='Output directory, defaults to "<project_name>.contents". '
                                          'If exists, all files in it will be deleted first.')
    parser.add_argument('--log_level', default='WARN', help='Set log level, one of WARN (default), INFO, DEBUG.')

    args = parser.parse_args()

    logging.basicConfig(level=args.log_level)

    vex_file = Path(args.vex_file)
    if not vex_file.is_file():
        raise ValueError(f'vex_file {vex_file} does not exist.')
    if not vex_file.parts[-1].endswith('.vex'):
        raise ValueError(f'vex_file {vex_file} must end with ".vex".')

    if not args.out_dir:
        *vex_dirs, vex_fn = vex_file.parts
        assert vex_fn.endswith('.vex')
        out_dir = Path(*vex_dirs, vex_fn[:-4] + '.contents')
    else:
        out_dir = args.out_dir
        pass

    execute(vex_file, out_dir)
    pass

















