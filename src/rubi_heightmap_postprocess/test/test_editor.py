"""Opt-in desktop smoke test; no ROS graph, real maps or actual user dialogs."""
import os
from pathlib import Path
import sys
import time

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from test_repair import floor, patch, snapshot_file
from rubi_heightmap_postprocess.repair import RepairSession


@pytest.mark.skipif(os.environ.get('RUBI_GUI_TEST') != '1', reason='Opt-in Tk desktop test')
def test_desktop_workflow(tmp_path, monkeypatch):
    from matplotlib.backend_bases import MouseEvent
    from rubi_heightmap_postprocess import editor

    notices = []
    monkeypatch.setattr(editor.messagebox, 'askyesno', lambda *a, **k: True)
    monkeypatch.setattr(editor.messagebox, 'showinfo', lambda *a, **k: notices.append(a))
    monkeypatch.setattr(editor.messagebox, 'showwarning', lambda *a, **k: pytest.fail(str(a)))
    z = floor()
    mask = patch(z.shape)
    z[mask] += .1
    z[9, 10] = z[31, 41] = np.nan
    session = RepairSession(snapshot_file(tmp_path, z))
    root = editor.tk.Tk()
    root.withdraw()
    app = editor.Editor(root, session, tmp_path/'exports')

    def wait_job():
        deadline = time.monotonic()+15
        while app.busy and time.monotonic() < deadline:
            root.update()
            time.sleep(.01)
        assert not app.busy, 'GUI worker failed to finish'
        root.update_idletasks()

    try:
        app.canvas.draw()
        x0, _, y0, _ = app.extent
        app.mode.set('Rectangle')
        app.set_mode()
        # Real Matplotlib mouse callbacks on the right panel (no desktop clicks).
        for event, x, y in [('button_press_event', x0+24*.05, y0+18*.05),
                            ('motion_notify_event', x0+29*.05, y0+23*.05),
                            ('button_release_event', x0+29*.05, y0+23*.05)]:
            px, py = app.right.transData.transform((x, y))
            app.canvas.callbacks.process(event, MouseEvent(event, app.canvas, px, py, button=1))
        np.testing.assert_array_equal(app.selection, mask)
        app.preview_region()
        wait_job()
        assert app.pending is not None
        np.testing.assert_array_equal(session.working, z)
        app.apply()
        np.testing.assert_allclose(session.working[mask], floor()[mask], atol=1e-12)
        app.undo()
        np.testing.assert_array_equal(session.working, z)
        app.redo()
        app.clear()
        app.mode.set('Polygon')
        app.set_mode()
        app.select([(x0+23*.05, y0+17*.05), (x0+30*.05, y0+17*.05),
                    (x0+30*.05, y0+24*.05), (x0+23*.05, y0+24*.05)])
        assert app.selection[mask].all()
        app.clear()
        app.mode.set('View')
        app.set_mode()
        app.preview_holes()
        wait_job()
        assert app.pending.details['accepted_holes'] == 2
        app.visit_hole(1)
        app.only_hole()
        assert len(app.pending.indices) == 1
        app.apply()
        assert np.count_nonzero(np.isfinite(session.working) & ~np.isfinite(z)) == 1
        app.delta_enabled.set(True)
        app.refresh()
        app.canvas.draw()
        app.delta_enabled.set(False)
        app.full_view()
        app.refresh()
        app.save()
        wait_job()
        saved = list((tmp_path/'exports').glob('*/repair.json'))
        assert len(saved) == 1
        loaded = RepairSession.load(saved[0].parent)
        np.testing.assert_array_equal(loaded.working, session.working)
        assert session.revision == app.saved_revision
        assert notices
    finally:
        app.close()


@pytest.mark.skipif(os.environ.get('RUBI_GUI_TEST') != '1', reason='Opt-in Tk desktop test')
def test_manual_floor_and_inline_rejection(tmp_path, monkeypatch):
    from rubi_heightmap_postprocess import editor
    monkeypatch.setattr(editor.messagebox, 'askyesno', lambda *a, **k: True)
    monkeypatch.setattr(editor.messagebox, 'showwarning', lambda *a, **k: pytest.fail(str(a)))
    z = floor()
    mask = patch(z.shape)
    z[mask] += .12
    z[20, 23] += .04
    session = RepairSession(snapshot_file(tmp_path, z))
    root = editor.tk.Tk()
    root.withdraw()
    app = editor.Editor(root, session, tmp_path/'exports')
    x0, _, y0, _ = app.extent

    def rect(c0, r0, c1, r1):
        return [(x0+c0*.05, y0+r0*.05), (x0+c1*.05, y0+r0*.05),
                (x0+c1*.05, y0+r1*.05), (x0+c0*.05, y0+r1*.05)]

    def wait_job():
        deadline = time.monotonic()+15
        while app.busy and time.monotonic() < deadline:
            root.update()
            time.sleep(.01)
        assert not app.busy

    try:
        app.select(rect(24, 18, 29, 23))
        app.preview_region()
        wait_job()
        assert app.pending is None
        assert app.bad_indices.size > 0
        assert 'FLOOR support' in app.feedback.get()
        app.selection_role.set('Floor')
        app.set_role()
        app.select(rect(17, 15, 21, 26))
        app.select(rect(32, 15, 36, 26))
        assert len(app.support_vertices) == 2
        assert app.selection[mask].all()
        app.preview_region()
        wait_job()
        assert app.pending.details['fit']['support_mode'] == 'manual_floor'
        assert not app.bad_indices.size
        app.canvas.draw()
        app.apply()
        np.testing.assert_allclose(session.working[mask], floor()[mask], atol=1e-12)
        np.testing.assert_array_equal(session.working[~mask], z[~mask])
        app.clear_floor()
        assert not app.support_mask.any()
        assert app.selection is not None
        app.undo()
        np.testing.assert_array_equal(session.working, z)
        app.selection_role.set('Artifact')
        app.set_role()
        app.select(rect(24, 18, 29, 23))
        assert not app.support_vertices
    finally:
        app.close()
