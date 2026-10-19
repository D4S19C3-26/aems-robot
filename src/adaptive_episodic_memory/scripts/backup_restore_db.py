#!/usr/bin/env python3
"""
AEMS Database Backup & Restore Utility
=======================================
FR-20 / Step 7 — Provides hot backup (SQLite online backup API) and restore
of the episodic memory database without taking the system offline.

Usage:
  Backup:   python3 backup_restore_db.py backup [--output PATH]
  Restore:  python3 backup_restore_db.py restore --input PATH [--confirm]
  List:     python3 backup_restore_db.py list [--backup-dir DIR]

Defaults:
  DB_PATH:       ~/aems_ws/aems_memory.db
  BACKUP_DIR:    ~/aems_ws/backups/
"""

import argparse
import glob
import os
import shutil
import sqlite3
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

DEFAULT_DB         = os.path.expanduser('~/aems_ws/aems_memory.db')
DEFAULT_BACKUP_DIR = os.path.expanduser('~/aems_ws/backups')
TIMESTAMP_FMT      = '%Y%m%d_%H%M%S'


def _ts() -> str:
    return datetime.now(timezone.utc).strftime(TIMESTAMP_FMT)


def do_backup(db_path: str, backup_dir: str, output: str = None) -> str:
    db_path = os.path.expanduser(db_path)
    if not os.path.exists(db_path):
        print(f'[ERROR] Database not found: {db_path}', file=sys.stderr)
        sys.exit(1)
    os.makedirs(backup_dir, exist_ok=True)
    dest = os.path.expanduser(output) if output else os.path.join(backup_dir, f'aems_memory_{_ts()}.db')
    t0 = time.time()
    src_conn = sqlite3.connect(db_path)
    dst_conn = sqlite3.connect(dest)
    try:
        src_conn.backup(dst_conn, pages=100, progress=_backup_progress)
    finally:
        src_conn.close()
        dst_conn.close()
    elapsed = time.time() - t0
    size_kb = os.path.getsize(dest) / 1024
    print(f'\n[OK] Backup complete -> {dest}  ({size_kb:.1f} KB, {elapsed:.2f}s)')
    return dest


def _backup_progress(status, remaining, total):
    if total > 0:
        pct = int((total - remaining) / total * 100)
        print(f'\r  Backup: {pct:3d}%  ({total-remaining}/{total} pages)', end='', flush=True)


def do_restore(db_path: str, input_path: str, confirm: bool = False):
    db_path  = os.path.expanduser(db_path)
    src_path = os.path.expanduser(input_path)
    if not os.path.exists(src_path):
        print(f'[ERROR] Backup file not found: {src_path}', file=sys.stderr)
        sys.exit(1)
    try:
        test = sqlite3.connect(src_path)
        test.execute('SELECT COUNT(*) FROM sqlite_master')
        test.close()
    except Exception as e:
        print(f'[ERROR] Backup file is not a valid SQLite database: {e}', file=sys.stderr)
        sys.exit(1)
    if not confirm:
        ans = input(f'[CONFIRM] Overwrite {db_path} with {src_path}?\nType YES to proceed: ')
        if ans.strip() != 'YES':
            print('[ABORTED] Restore cancelled.')
            sys.exit(0)
    if os.path.exists(db_path):
        safety_snap = db_path + f'.pre_restore_{_ts()}'
        shutil.copy2(db_path, safety_snap)
        print(f'[INFO] Safety snapshot saved -> {safety_snap}')
    t0 = time.time()
    src_conn = sqlite3.connect(src_path)
    dst_conn = sqlite3.connect(db_path)
    try:
        src_conn.backup(dst_conn)
    finally:
        src_conn.close()
        dst_conn.close()
    elapsed = time.time() - t0
    size_kb = os.path.getsize(db_path) / 1024
    print(f'[OK] Restore complete -> {db_path}  ({size_kb:.1f} KB, {elapsed:.2f}s)')


def do_list(backup_dir: str):
    backup_dir = os.path.expanduser(backup_dir)
    pattern = os.path.join(backup_dir, 'aems_memory_*.db')
    files = sorted(glob.glob(pattern), reverse=True)
    if not files:
        print(f'No backups found in {backup_dir}')
        return
    print(f'{"FILE":<50}  {"SIZE":>8}  MODIFIED')
    print('-' * 80)
    for f in files:
        size_kb = os.path.getsize(f) / 1024
        mtime = datetime.fromtimestamp(os.path.getmtime(f)).strftime('%Y-%m-%d %H:%M:%S')
        print(f'{os.path.basename(f):<50}  {size_kb:>7.1f}K  {mtime}')


def main():
    parser = argparse.ArgumentParser(description='AEMS Database Backup & Restore (FR-20, Step 7)')
    parser.add_argument('--db', default=DEFAULT_DB)
    parser.add_argument('--backup-dir', default=DEFAULT_BACKUP_DIR)
    sub = parser.add_subparsers(dest='command', required=True)

    p_b = sub.add_parser('backup')
    p_b.add_argument('--output', default=None)

    p_r = sub.add_parser('restore')
    p_r.add_argument('--input', required=True)
    p_r.add_argument('--confirm', action='store_true')

    sub.add_parser('list')

    args = parser.parse_args()
    if args.command == 'backup':
        do_backup(args.db, args.backup_dir, args.output)
    elif args.command == 'restore':
        do_restore(args.db, args.input, args.confirm)
    elif args.command == 'list':
        do_list(args.backup_dir)


if __name__ == '__main__':
    main()
