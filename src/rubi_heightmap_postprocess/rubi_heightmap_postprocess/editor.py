"""Desktop offline repair editor. ROS nodes and original maps are never modified."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path
from queue import Empty, Queue
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.colors import ListedColormap
from matplotlib.figure import Figure
from matplotlib.widgets import PolygonSelector, RectangleSelector

from .map_io import read_snapshot
from .repair import Proposal, RepairSession, RepairSettings, SupportFitError, polygon_mask


class Editor:
    def __init__(self, root, session, output_root=None):
        self.root, self.session = root, session
        self.output_root = Path(output_root) if output_root else session.snapshot.source.parent/'repairs'
        self.pending = None
        self.selection = None
        self.vertices = None
        self.support_mask = np.zeros(session.raw.shape, bool)
        self.support_vertices = []
        self.fit_support_indices = np.empty(0, np.int64)
        self.bad_indices = np.empty(0, np.int64)
        self.selector = None
        self.hole_index = -1
        self.busy = False
        self.saved_revision = session.revision
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.results = Queue()
        self.mode = tk.StringVar(root, 'View')
        self.selection_role = tk.StringVar(root, 'Artifact')
        self.support_summary = tk.StringVar(root, 'Floor: automatic ring (or select clean patches).')
        self.feedback = tk.StringVar(root, 'Pick the artifact first. If the automatic ring fails, select clean floor on both sides.')
        self.overlay_enabled = tk.BooleanVar(root, True)
        self.delta_enabled = tk.BooleanVar(root, False)
        self.status = tk.StringVar(root, 'Zoom first, then select a small artifact on the RIGHT map.')
        root.title('RUBI Height Repair v2 - MANUAL FLOOR SUPPORT')
        root.geometry('1440x900')
        root.minsize(1100, 740)
        root.protocol('WM_DELETE_WINDOW', self.close)
        ttk.Label(root, text=f'Original: {session.snapshot.source}', padding=8).pack(fill='x')
        outer = ttk.Frame(root)
        outer.pack(fill='both', expand=True)
        sidebar = ttk.Frame(outer, padding=8, width=290)
        sidebar.pack(side='right', fill='y')
        tabs = ttk.Notebook(sidebar)
        tabs.pack(fill='both', expand=True)
        side = ttk.Frame(tabs, padding=8)
        options = ttk.Frame(tabs, padding=8)
        tabs.add(side, text='Artifact repair')
        tabs.add(options, text='Holes / limits')
        canvas_frame = ttk.Frame(outer)
        canvas_frame.pack(side='left', fill='both', expand=True)
        self.figure = Figure(figsize=(10, 7), constrained_layout=True)
        self.left, self.right = self.figure.subplots(1, 2, sharex=True, sharey=True)
        snap = session.snapshot
        r = snap.resolution
        x0, y0 = snap.origin[0]+snap.offset[0]*r, snap.origin[1]+snap.offset[1]*r
        self.extent = (x0, x0+session.raw.shape[1]*r, y0, y0+session.raw.shape[0]*r)
        finite = session.raw[np.isfinite(session.raw)]
        lo, hi = np.quantile(finite, [.02, .98])
        self.limits = (float(lo), float(max(hi, lo+.05)))
        cmap = __import__('matplotlib').cm.get_cmap('viridis').copy()
        cmap.set_bad('#202832')
        self.height_cmap = cmap
        self.images = []
        for ax, z, title in ((self.left, session.raw, 'Original (immutable)'),
                             (self.right, session.working, 'Working copy')):
            self.images.append(ax.imshow(z, origin='lower', extent=self.extent,
                                         interpolation='nearest', cmap=cmap,
                                         vmin=self.limits[0], vmax=self.limits[1]))
            ax.set_title(title)
            ax.set_xlabel(f'{snap.frame} X [m]')
            ax.set_ylabel(f'{snap.frame} Y [m]')
            ax.set_facecolor('#202832')
        self.colorbar = self.figure.colorbar(self.images[0], ax=[self.left, self.right], shrink=.6)
        self.colorbar.set_label('Height [m] (2-98 percentile display range)')
        self.overlay = self.right.imshow(np.ma.masked_all(session.raw.shape), origin='lower',
                                          extent=self.extent, interpolation='nearest',
                                          cmap=ListedColormap(['#ffad33', '#00e6ef', '#ff43d0']),
                                          vmin=1, vmax=3, alpha=.65)
        self.outlines = [ax.plot([], [], color='#ff43d0', linewidth=1.4)[0]
                         for ax in (self.left, self.right)]
        self.support_overlays = [ax.imshow(np.ma.masked_all(session.raw.shape), origin='lower',
                                          extent=self.extent, interpolation='nearest',
                                          cmap=ListedColormap(['#63ee6b']), vmin=0, vmax=1, alpha=.45)
                                 for ax in (self.left, self.right)]
        self.bad_marks = [ax.plot([], [], 'x', color='#ff3434', markersize=9, markeredgewidth=2)[0]
                          for ax in (self.left, self.right)]
        self.canvas = FigureCanvasTkAgg(self.figure, master=canvas_frame)
        self.toolbar = NavigationToolbar2Tk(self.canvas, canvas_frame, pack_toolbar=False)
        self.toolbar.pack(side='bottom', fill='x')
        self.canvas.get_tk_widget().pack(fill='both', expand=True)
        self.canvas.mpl_connect('scroll_event', self.scroll)

        ttk.Label(side, text='1. Select on the RIGHT map', font=('', 12, 'bold')).pack(anchor='w')
        self.mode_buttons = []
        mode_row = ttk.Frame(side)
        mode_row.pack(fill='x')
        for mode in ('View', 'Rectangle', 'Polygon'):
            button = ttk.Radiobutton(mode_row, text=mode, variable=self.mode, value=mode,
                                     command=self.set_mode)
            button.pack(side='left')
            self.mode_buttons.append(button)
        ttk.Label(side, text='Wheel: zoom. View + toolbar: pan.\nPolygon: click vertices, then first point.',
                  wraplength=265).pack(anchor='w', pady=6)
        for value, label in [('Artifact', 'A. Artifact to erase (magenta)'),
                             ('Floor', 'B. Add clean floor patches (green)')]:
            button = ttk.Radiobutton(side, text=label, variable=self.selection_role, value=value,
                                     command=self.set_role)
            button.pack(anchor='w')
            self.mode_buttons.append(button)
        ttk.Label(side, text='A replaces the target and clears old floor.\nB adds patches: draw on BOTH sides of A.\nUse nearby floor, not other footprints.',
                  wraplength=265).pack(anchor='w', pady=5)
        ttk.Label(side, textvariable=self.support_summary, wraplength=265).pack(anchor='w', pady=5)
        self.buttons = []
        self.button(side, 'Clear floor only / use automatic ring', self.clear_floor)
        navigation_row = ttk.Frame(side)
        navigation_row.pack(fill='x')
        self.button(navigation_row, 'Full view', self.full_view, horizontal=True)
        self.button(navigation_row, 'Clear selection', self.clear, horizontal=True)
        ttk.Separator(side).pack(fill='x', pady=8)
        ttk.Label(side, text='2. Preview (no changes yet)', font=('', 12, 'bold')).pack(anchor='w')
        self.button(side, 'Preview selected patch repair', self.preview_region)
        ttk.Label(side, textvariable=self.feedback, wraplength=265, foreground='#8b3d13').pack(anchor='w', pady=8)
        ttk.Label(options, text='Fit limits (metres)', font=('', 12, 'bold')).pack(anchor='w')
        self.fields = {}
        settings = RepairSettings()
        for key, title in (('ring_m', 'Floor support ring [m]'), ('max_rms_m', 'Max floor RMS [m]'),
                           ('max_correction_m', 'Max height correction [m]')):
            row = ttk.Frame(options)
            row.pack(fill='x', pady=2)
            ttk.Label(row, text=title).pack(side='left')
            value = tk.StringVar(root, str(getattr(settings, key)))
            ttk.Entry(row, textvariable=value, width=7).pack(side='right')
            self.fields[key] = value
        ttk.Label(options, text='Automatic ring only: support ring width.\nManual floor: within 1 m, surrounds target.\nBoth: clean original support and fit checks.',
                  wraplength=265).pack(anchor='w', pady=8)
        self.button(options, 'Find small holes (selection or all)', self.preview_holes)
        hole_row = ttk.Frame(options)
        hole_row.pack(fill='x')
        self.button(hole_row, 'Previous hole', lambda: self.visit_hole(-1), horizontal=True)
        self.button(hole_row, 'Next hole', lambda: self.visit_hole(1), horizontal=True)
        self.button(options, 'Keep only this hole in preview', self.only_hole)
        ttk.Label(options, text='Holes: <=4 cells, <=10 cm across.\nHoles always use the automatic ring.\nApply accepts ALL current preview cells.\nNo automatic person detection.',
                  wraplength=265).pack(anchor='w', pady=5)
        ttk.Separator(side).pack(fill='x', pady=8)
        ttk.Label(side, text='3. Review / apply / save', font=('', 12, 'bold')).pack(anchor='w')
        self.button(side, 'Apply preview', self.apply)
        history_row = ttk.Frame(side)
        history_row.pack(fill='x')
        self.button(history_row, 'Undo  [Ctrl+Z]', self.undo, horizontal=True)
        self.button(history_row, 'Redo  [Ctrl+Y]', self.redo, horizontal=True)
        self.button(side, 'Save review copy  [Ctrl+S]', self.save)
        ttk.Checkbutton(side, text='Show edit masks', variable=self.overlay_enabled,
                        command=self.refresh).pack(anchor='w', pady=3)
        ttk.Checkbutton(side, text='Show height difference [m]', variable=self.delta_enabled,
                        command=self.refresh).pack(anchor='w')
        ttk.Label(side, text='Green: floor support. Red X: conflicting floor.\nOrange: edited. Cyan: inferred.\nReview only - NOT navigation approval.',
                  wraplength=265).pack(anchor='w', pady=8)
        ttk.Label(root, textvariable=self.status, wraplength=1350, padding=8).pack(fill='x')
        root.bind('<Control-z>', lambda _: self.undo())
        root.bind('<Control-y>', lambda _: self.redo())
        root.bind('<Control-s>', lambda _: self.save())
        root.bind('<Escape>', lambda _: self.clear())
        self.poll_id = root.after(100, self.poll)
        self.refresh()

    def button(self, parent, text, command, horizontal=False):
        button = ttk.Button(parent, text=text, command=command)
        button.pack(fill='x', pady=2, side='left' if horizontal else 'top', expand=horizontal)
        self.buttons.append(button)

    def settings(self):
        settings = RepairSettings(**{key: float(value.get()) for key, value in self.fields.items()})
        settings.validate()
        return settings

    def set_role(self):
        if self.busy:
            return
        if self.mode.get() == 'View':
            self.mode.set('Rectangle')
        self.set_mode()

    def set_mode(self):
        if self.busy:
            return
        if self.selector is not None:
            self.selector.set_active(False)
            self.selector.set_visible(False)
            self.selector.disconnect_events()
            for artist in self.selector.artists:
                artist.remove()
            self.selector = None
        if self.mode.get() != 'View':
            if self.toolbar.mode == 'pan/zoom':
                self.toolbar.pan()
            elif self.toolbar.mode == 'zoom rect':
                self.toolbar.zoom()
        if self.mode.get() == 'Rectangle':
            self.selector = RectangleSelector(self.right, self.rectangle, useblit=True,
                                              button=[1], interactive=self.selection_role.get() == 'Artifact',
                                              minspanx=3, minspany=3,
                                              spancoords='pixels')
        elif self.mode.get() == 'Polygon':
            self.selector = PolygonSelector(self.right, self.select, useblit=True)
        self.canvas.draw_idle()

    def rectangle(self, start, end):
        if None in (start.xdata, start.ydata, end.xdata, end.ydata):
            return
        x0, x1 = sorted((start.xdata, end.xdata))
        y0, y1 = sorted((start.ydata, end.ydata))
        self.select([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])

    def select(self, vertices):
        if self.busy:
            return
        self.fit_support_indices = self.bad_indices = np.empty(0, np.int64)
        if self.selection_role.get() == 'Floor':
            if self.selection is None:
                self.feedback.set('Select A. Artifact first, then add floor patches with B.')
                return
            self.support_mask |= polygon_mask(self.session.snapshot, vertices)
            self.support_vertices.append([list(v) for v in vertices])
            self.pending = None
            self.hole_index = -1
            count = int(self.usable_support().sum())
            self.support_summary.set(f'Floor: {len(self.support_vertices)} manual patches, {count} usable cells.')
            self.feedback.set('Draw clean floor on the other side too, then Preview. Only the magenta artifact will change.')
            self.status.set('Added floor support, not an edit. Unknown/edited cells and the artifact are excluded.')
            self.refresh()
            if self.mode.get() == 'Polygon':
                self.root.after_idle(self.set_mode)
            return
        self.vertices = [list(v) for v in vertices]
        self.selection = polygon_mask(self.session.snapshot, vertices)
        self.support_mask[:] = False
        self.support_vertices = []
        self.support_summary.set('Floor: automatic ring (or select clean patches).')
        self.pending = None
        self.hole_index = -1
        line = np.asarray(vertices + [vertices[0]]) if isinstance(vertices, list) else np.vstack((vertices, vertices[0]))
        for outline in self.outlines:
            outline.set_data(line[:, 0], line[:, 1])
        self.status.set(f'Selected {self.selection.sum()} cells. Preview patch repair, or find holes inside this region.')
        self.feedback.set('Target selected. Preview automatic fit, or choose B and draw clean floor on both sides.')
        self.refresh()

    def usable_support(self):
        support = self.support_mask & np.isfinite(self.session.raw) & (self.session.methods == 0)
        if self.selection is not None:
            support &= ~self.selection
        return support

    def clear_floor(self):
        if self.busy:
            return
        self.support_mask[:] = False
        self.support_vertices = []
        self.fit_support_indices = self.bad_indices = np.empty(0, np.int64)
        self.pending = None
        self.support_summary.set('Floor: automatic ring (or select clean patches).')
        self.feedback.set('Floor patches cleared; the artifact selection is unchanged.')
        self.set_mode()
        self.refresh()

    def full_view(self):
        self.left.set_xlim(self.extent[:2])
        self.left.set_ylim(self.extent[2:])
        self.canvas.draw_idle()

    def scroll(self, event):
        if event.inaxes not in (self.left, self.right) or event.xdata is None:
            return
        factor = .75 if event.button == 'up' else 1/.75
        for limits, center, setter in ((event.inaxes.get_xlim(), event.xdata, self.left.set_xlim),
                                      (event.inaxes.get_ylim(), event.ydata, self.left.set_ylim)):
            setter(center+(np.asarray(limits)-center)*factor)
        self.canvas.draw_idle()

    def clear(self):
        if self.busy:
            return
        self.pending = self.selection = self.vertices = None
        self.support_mask[:] = False
        self.support_vertices = []
        self.fit_support_indices = self.bad_indices = np.empty(0, np.int64)
        self.selection_role.set('Artifact')
        self.support_summary.set('Floor: automatic ring (or select clean patches).')
        self.feedback.set('Select A. Artifact; then use B to add clean floor if needed.')
        self.hole_index = -1
        for outline in self.outlines:
            outline.set_data([], [])
        self.set_mode()
        self.status.set('Selection and preview cleared. Applied edits are unchanged.')
        self.refresh()

    def job(self, label, function, done):
        if self.busy:
            return
        self.busy = True
        self.status.set(label)
        for button in self.buttons + self.mode_buttons:
            button.state(['disabled'])
        if self.selector is not None:
            self.selector.set_active(False)
        def work():
            try:
                self.results.put((done, function(), None))
            except Exception as error:
                self.results.put((done, None, error))
        self.pool.submit(work)

    def poll(self):
        try:
            done, result, error = self.results.get_nowait()
        except Empty:
            pass
        else:
            self.busy = False
            for button in self.buttons + self.mode_buttons:
                button.state(['!disabled'])
            if self.selector is not None:
                self.selector.set_active(True)
            if error is not None:
                self.pending = None
                self.status.set('Not applied: '+str(error))
                self.feedback.set(str(error))
                if isinstance(error, SupportFitError):
                    self.fit_support_indices = error.support_indices
                    self.bad_indices = error.outlier_indices
                # Expected fit rejection stays inline so it does not block selection.
                if not isinstance(error, ValueError):
                    messagebox.showwarning('Operation failed', str(error), parent=self.root)
                self.refresh()
            else:
                done(result)
        self.poll_id = self.root.after(100, self.poll)

    def preview_region(self):
        if self.busy:
            return
        try:
            settings = self.settings()
            if self.selection is None:
                raise ValueError('Choose Rectangle or Polygon and select the artifact on the RIGHT map.')
        except ValueError as error:
            messagebox.showinfo('Selection/settings', str(error), parent=self.root)
            return
        self.pending = None
        self.bad_indices = np.empty(0, np.int64)
        support = self.support_mask.copy() if self.support_vertices else None
        self.feedback.set('Checking selected floor...' if support is not None else 'Checking automatic ring...')
        self.job('Checking original floor support...',
                 lambda: self.session.preview_region(self.selection, settings, self.vertices,
                             support_mask=support, support_vertices=self.support_vertices), self.show_proposal)

    def preview_holes(self):
        if self.busy:
            return
        try:
            settings = self.settings()
        except ValueError as error:
            messagebox.showwarning('Settings', str(error), parent=self.root)
            return
        self.pending = None
        self.job('Finding small interior holes; no edits applied...',
                 lambda: self.session.preview_holes(settings, self.selection), self.show_proposal)

    def show_proposal(self, proposal):
        self.pending = proposal if len(proposal.indices) else None
        self.hole_index = -1
        details = proposal.details
        self.bad_indices = np.empty(0, np.int64)
        if proposal.method == 2:
            self.status.set(f"Preview: {details['accepted_holes']} holes, {len(proposal.indices)} cells. "
                            'Use Next hole to inspect; Apply accepts ALL preview cells. '
                            f"Rejected: {sum(details['rejected_holes'].values())}.")
        else:
            fit = details['fit']
            self.fit_support_indices = np.asarray(fit['support_indices'], dtype=np.int64)
            self.feedback.set(f"{fit['support_mode']}: RMS {fit['support_rms_m']*1000:.2f} mm, "
                              f"peak {fit['support_peak_m']*1000:.2f} mm. "
                              f"Artifact correction up to {fit['maximum_observed_change_m']*1000:.1f} mm. "
                              'Review surface before Apply.')
            self.status.set(f"Preview only: {len(proposal.indices)} cells; support RMS "
                            f"{fit['support_rms_m']*1000:.2f} mm; max correction "
                            f"{fit['maximum_observed_change_m']*1000:.1f} mm. Review before Apply.")
        self.refresh()

    def visit_hole(self, direction):
        if self.busy or self.pending is None or self.pending.method != 2:
            return
        fits = self.pending.details['fits']
        self.hole_index = (self.hole_index+direction) % len(fits)
        y0, y1, x0, x1 = fits[self.hole_index]['bbox_crop_yx']
        r = self.session.snapshot.resolution
        x, y = self.extent[0]+(x0+x1)*r/2, self.extent[2]+(y0+y1)*r/2
        self.left.set_xlim(x-.7, x+.7)
        self.left.set_ylim(y-.7, y+.7)
        self.status.set(f'Hole {self.hole_index+1}/{len(fits)} at ({x:.2f}, {y:.2f}) m. '
                        'Use Keep only this hole to exclude the other candidates.')
        self.canvas.draw_idle()

    def only_hole(self):
        if self.busy or self.pending is None or self.pending.method != 2 or self.hole_index < 0:
            return
        fit = self.pending.details['fits'][self.hole_index]
        y0, y1, x0, x1 = fit['bbox_crop_yx']
        yy, xx = np.unravel_index(self.pending.indices, self.session.raw.shape)
        keep = (yy >= y0) & (yy < y1) & (xx >= x0) & (xx < x1)
        details = dict(self.pending.details, fits=[fit], accepted_holes=1, cells=int(keep.sum()),
                       newly_filled_cells=int(keep.sum()), user_selected_one_hole=True)
        self.show_proposal(Proposal(self.pending.revision, self.pending.indices[keep],
                                    self.pending.values[keep], 2, details))

    def apply(self):
        if self.busy or self.pending is None:
            return
        if not messagebox.askyesno('Confirm inferred terrain',
                                  f'Apply {len(self.pending.indices)} preview cells?\n'
                                  'Only proceed if this is known floor / a confirmed artifact.\n'
                                  'This does not approve the map for navigation.', parent=self.root):
            return
        self.session.apply(self.pending)
        self.pending = None
        self.status.set('Applied to working copy only. Undo is available; save a review copy when ready.')
        self.feedback.set('Applied to working copy only. Undo available. Save before closing.')
        self.refresh()

    def undo(self):
        if self.busy:
            return
        self.pending = None
        changed = self.session.undo()
        self.status.set('Undone.' if changed else 'No undo history in this session.')
        self.refresh()

    def redo(self):
        if self.busy:
            return
        self.pending = None
        changed = self.session.redo()
        self.status.set('Redone.' if changed else 'No redo history.')
        self.refresh()

    def save(self):
        if self.busy:
            return
        if self.pending is not None:
            messagebox.showinfo('Unapplied preview', 'Apply or cancel the preview before saving.', parent=self.root)
            return
        destination = self.output_root / ('repair_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        def done(path):
            self.saved_revision = self.session.revision
            self.status.set(f'Saved review bundle: {path}')
            messagebox.showinfo('Review copy saved', str(path)+'\n\nrepaired_preview.pcd / repair_layers.npz: includes fills.'
                                '\nobserved_only/map.rghm: original observation mask preserved.'
                                '\nNo automatic navigation export or approval.', parent=self.root)
        self.job('Saving a NEW copy; original remains unchanged...', lambda: self.session.save(destination), done)

    def refresh(self):
        z = self.session.working
        if self.pending is not None:
            z = z.copy()
            z.flat[self.pending.indices] = self.pending.values
        if self.delta_enabled.get():
            self.images[1].set_data(z-self.session.raw)
            self.images[1].set_cmap('RdBu_r')
            self.images[1].set_clim(-.10, .10)
            self.right.set_title('Difference [m], -0.10 to +0.10 (fills: mask only)')
        else:
            self.images[1].set_data(z)
            self.images[1].set_cmap(self.height_cmap)
            self.images[1].set_clim(*self.limits)
            self.right.set_title('PREVIEW - not applied' if self.pending is not None else 'Working copy')
        codes = np.zeros(z.shape, np.uint8)
        codes[(self.session.methods != 0) & np.isfinite(self.session.raw)] = 1
        codes[~np.isfinite(self.session.raw) & np.isfinite(self.session.working)] = 2
        if self.pending is not None:
            codes.flat[self.pending.indices] = 3
        self.overlay.set_data(np.ma.masked_equal(codes, 0))
        self.overlay.set_visible(self.overlay_enabled.get())
        support = self.usable_support()
        if not self.support_vertices:
            support.flat[self.fit_support_indices] = True
        for overlay in self.support_overlays:
            overlay.set_data(np.ma.masked_where(~support, support))
            overlay.set_visible(self.overlay_enabled.get())
        yy, xx = np.unravel_index(self.bad_indices, z.shape)
        r = self.session.snapshot.resolution
        for mark in self.bad_marks:
            mark.set_data(self.extent[0]+(xx+.5)*r, self.extent[2]+(yy+.5)*r)
        self.canvas.draw_idle()

    def close(self):
        if self.busy:
            messagebox.showinfo('Please wait', 'An offline operation is still running.', parent=self.root)
            return
        if self.session.revision != self.saved_revision or self.pending is not None:
            if not messagebox.askyesno('Discard unsaved work?', 'Close without saving current edits/preview?', parent=self.root):
                return
        self.root.after_cancel(self.poll_id)
        self.pool.shutdown(wait=False)
        self.root.destroy()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--input', type=Path, help='Original map.rghm (never overwritten)')
    group.add_argument('--session', type=Path, help='Reopen a saved repair bundle')
    parser.add_argument('--output-root', type=Path, help='Parent for uniquely named review copies')
    args = parser.parse_args(argv)
    root = tk.Tk()
    root.withdraw()
    try:
        if args.session:
            session = RepairSession.load(args.session)
        else:
            source = args.input or filedialog.askopenfilename(parent=root, title='Open ORIGINAL height map',
                        initialdir=str(Path.cwd()/'maps/raw'), filetypes=[('RUBI height map', '*.rghm')])
            if not source:
                root.destroy()
                return
            session = RepairSession(read_snapshot(source))
        Editor(root, session, args.output_root)
    except Exception as error:
        messagebox.showerror('Cannot open height editor', str(error), parent=root)
        root.destroy()
        raise
    root.deiconify()
    root.mainloop()


if __name__ == '__main__':
    main()
