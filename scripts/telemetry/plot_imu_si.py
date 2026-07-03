from plot_common import *
from telemetry import *

FIELDS = PACKET_DEFS[TelemetryId.IMU_SI].fields
GYRO_FIELDS = [f for f in FIELDS if f.startswith("g")]
ACC_FIELDS = [f for f in FIELDS if f.startswith("a")]

timestamps, data, lock = start_reader(TelemetryId.IMU_SI)

app, win = start_app()

gyro_plot = make_plot(win, "Gyroscope [rad/s]")
gyro_curves = make_curves(gyro_plot, GYRO_FIELDS)

win.nextRow()

acc_plot = make_plot(win, "Accelerometer [m/s^2]")
acc_curves = make_curves(acc_plot, ACC_FIELDS)


def update_plot():
    with lock:
        if len(timestamps) < 2:
            return
        x = list(timestamps)
        y = {f: list(data[f]) for f in FIELDS}
    for f, c in gyro_curves.items():
        c.setData(x, y[f])
    for f, c in acc_curves.items():
        c.setData(x, y[f])


timer = start_timer(update_plot)
app.exec()
