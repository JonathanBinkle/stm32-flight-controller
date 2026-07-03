from typing import Iterable, Callable
from pyqtgraph.Qt import QtCore, QtWidgets
import pyqtgraph as pg
from pyqtgraph.widgets.GraphicsLayoutWidget import GraphicsLayoutWidget

pg.setConfigOption("background", "w")
pg.setConfigOption("foreground", "k")


def start_app() -> tuple[QtWidgets.QApplication, GraphicsLayoutWidget]:
    app = QtWidgets.QApplication([])
    win = pg.GraphicsLayoutWidget(show=True, title="Quadcopter Telemetry")
    win.resize(1400, 900)
    return app, win


def make_plot(win: GraphicsLayoutWidget, title: str) -> pg.PlotWidget:
    plot = win.addPlot(title=title)
    plot.showGrid(x=True, y=True, alpha=0.2)
    plot.addLegend()
    return plot


def make_curves(
    plot: pg.PlotWidget, fields: Iterable[str]
) -> dict[str, pg.PlotDataItem]:
    curves: dict[str, pg.PlotDataItem] = {}

    for i, field in enumerate(fields):
        curves[field] = plot.plot(
            name=field,
            pen=pg.mkPen(pg.intColor(i), width=2),
        )

    return curves


def start_timer(callback: Callable[[], None], interval_ms: int = 50) -> QtCore.QTimer:
    timer = QtCore.QTimer()
    timer.timeout.connect(callback)
    timer.start(interval_ms)
    return timer
