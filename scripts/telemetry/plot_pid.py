from plot_common import *
from telemetry import *

FIELDS = (
    "desired",
    "actual",
    "pterm",
    "iterm",
    "dterm",
)

timestamps, data, lock = start_reader(TelemetryId.PID_PITCH)

app, win = start_app()

# Desired vs Actual
response_plot = make_plot(win, "PID Response")
response_plot.setLabel("left", "Angle", units="deg")
response_plot.setLabel("bottom", "Time", units="s")
response_curves = make_curves(response_plot, ("desired", "actual"))

# PID terms
win.nextRow()
terms_plot = make_plot(win, "PID Terms")
terms_plot.setLabel("left", "Output")
terms_plot.setLabel("bottom", "Time", units="s")
term_curves = make_curves(terms_plot, ("pterm", "iterm", "dterm"))


def update_plot():
    with lock:
        if len(timestamps) < 2:
            return

        x = list(timestamps)

        desired = list(data["desired"])
        actual = list(data["actual"])
        pterm = list(data["pterm"])
        iterm = list(data["iterm"])
        dterm = list(data["dterm"])

    response_curves["desired"].setData(x, desired)
    response_curves["actual"].setData(x, actual)

    term_curves["pterm"].setData(x, pterm)
    term_curves["iterm"].setData(x, iterm)
    term_curves["dterm"].setData(x, dterm)


timer = start_timer(update_plot)
app.exec()
