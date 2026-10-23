#!/usr/bin/env python3
"""Install native VS Code Watch expressions while Code is closed.
Preserves existing expressions and backs up the workspace database before editing.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import sqlite3
import uuid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true', help='Write the workspace database (default: preview only)')
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[1]
    names = json.loads((workspace / '.vscode/lqr-watch.json').read_text())['expressions']
    root = Path.home() / '.config/Code/User/workspaceStorage'
    matches = []
    for metadata in root.glob('*/workspace.json'):
        if json.loads(metadata.read_text()).get('folder') == workspace.as_uri():
            matches.append(metadata.parent / 'state.vscdb')
    if len(matches) != 1:
        raise SystemExit('Expected exactly one matching VS Code folder workspace; found %d' % len(matches))
    for comm in Path('/proc').glob('[0-9]*/comm'):
        try:
            running = comm.read_text().strip() in ('code', 'code-insiders')
        except OSError:
            continue
        if running:
            raise SystemExit('Close VS Code first so its in-memory state cannot overwrite this change.')
    database = matches[0]
    with sqlite3.connect(database.as_uri() + '?mode=ro', uri=True) as connection:
        row = connection.execute('SELECT value FROM ItemTable WHERE key=?', ('debug.watchexpressions',)).fetchone()
        watches = json.loads(row[0]) if row else []
        existing = {watch['name'] for watch in watches}
        additions = [dict(name=name, id=str(uuid.uuid4())) for name in names if name not in existing]
    print('Workspace:', workspace)
    print('Database:', database)
    print('Preserved watch count:', len(watches))
    print('Expressions to add:', json.dumps([a['name'] for a in additions], ensure_ascii=False))
    if not args.apply:
        return
    backup = database.with_name('state.vscdb.lqr-watch-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f') + '.backup')
    with sqlite3.connect(database) as connection:
        with sqlite3.connect(backup) as destination:
            connection.backup(destination)
        connection.execute('INSERT OR REPLACE INTO ItemTable(key,value) VALUES (?,?)',
                           ('debug.watchexpressions', json.dumps(watches + additions)))
        row = connection.execute('SELECT value FROM ItemTable WHERE key=?', ('workbench.view.debug.state',)).fetchone()
        views = json.loads(row[0]) if row else {}
        watch_view = views.setdefault('workbench.debug.watchExpressionsView', {})
        watch_view.update(collapsed=False, isHidden=False)
        connection.execute('INSERT OR REPLACE INTO ItemTable(key,value) VALUES (?,?)',
                           ('workbench.view.debug.state', json.dumps(views)))
    with sqlite3.connect(database.as_uri() + '?mode=ro', uri=True) as connection:
        stored = json.loads(connection.execute('SELECT value FROM ItemTable WHERE key=?',
                                               ('debug.watchexpressions',)).fetchone()[0])
        assert all(name in {watch['name'] for watch in stored} for name in names)
    print('Verified saved Watch expressions. Backup:', backup)


if __name__ == '__main__':
    main()
