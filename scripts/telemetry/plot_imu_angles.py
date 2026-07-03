from plot_common import *
from telemetry import *

# FIELDS = PACKET_DEFS[TelemetryId.IMU_ANGLES].fields
FIELDS = ("fused_pitch", "fused_roll")

timestamps, data, lock = start_reader(TelemetryId.IMU_ANGLES)

app, win = start_app()

plot = make_plot(win, "Roll / Pitch")
plot.setLabel("left", "Angle", units="deg")
plot.setLabel("bottom", "Time", units="s")
curves = make_curves(plot, FIELDS)


def update_plot():
    with lock:
        if len(timestamps) < 2:
            return
        x = list(timestamps)
        y = {f: list(data[f]) for f in FIELDS}
    for f, c in curves.items():
        c.setData(x, y[f])


timer = start_timer(update_plot)
app.exec()
