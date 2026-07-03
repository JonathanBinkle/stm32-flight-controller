from telemetry import connect, read_packet, Packet

conn = connect()

while True:
    pkt = read_packet(conn)

    print(f"\n[#{pkt.seqnum}] {pkt.id.name}")

    for k, v in pkt.fields.items():
        if isinstance(v, float):
            print(f"{k:20} {v:10.3f}")
        else:
            print(f"{k:20} {v}")
